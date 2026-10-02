import type { Contour, ContourWarning, Pt } from './types'
import {
  cleanPoints,
  dist,
  duplicateBins,
  findSelfIntersections,
  pathsOverlap,
  polygonArea,
  polylineLength,
  round3,
  toCCW,
  uid,
} from './geometry'

export type RawSub = { points: Pt[]; closed: boolean }

export type CleanupOptions = { toleranceMm: number; closeToleranceMm: number }

/** 重复路径分组：kept 保留者，removed 被合并掉的（反向/完全重叠） */
export type DuplicateGroup = { kept: Contour; removed: Contour[] }

export type CleanupReport = {
  input: number
  kept: number
  dropped: number
  autoClosed: number
  duplicates: number
  notClosed: number
  selfIntersect: number
  duplicateGroups: DuplicateGroup[]
  /** 每个轮廓的自交点数量 */
  selfIntersections: Record<string, number>
}

/** 重复比对容差（mm）：点到折线距离不超过该值即视为同一条线；放宽到 0.1 以容忍手绘抖动 */
export const DUPLICATE_MATCH_MM = 0.1

export function makeContour(points: Pt[], closed: boolean, warnings: ContourWarning[] = []): Contour {
  const pts = points.map((p) => ({ x: round3(p.x), y: round3(p.y) }))
  return {
    id: uid('c'),
    points: pts,
    closed,
    area: closed ? polygonArea(pts) : 0,
    length: polylineLength(pts, closed),
    holes: [],
    bridges: [],
    warnings: [...warnings],
  }
}

type Cand = { points: Pt[]; closed: boolean; warnings: ContourWarning[] }

/**
 * 路径清理：去重合点 / 共线点 → 闭合检查（近闭合自动闭合、未闭合打标）→
 * 重复路径合并（同一条线画了几遍就全部合为一条，含反向）→ 自交检测
 */
export function cleanupContours(
  raw: RawSub[],
  opts: CleanupOptions,
): { contours: Contour[]; report: CleanupReport } {
  const report: CleanupReport = {
    input: raw.length,
    kept: 0,
    dropped: 0,
    autoClosed: 0,
    duplicates: 0,
    notClosed: 0,
    selfIntersect: 0,
    duplicateGroups: [],
    selfIntersections: {},
  }

  const cands: Cand[] = []

  for (const sub of raw) {
    if (sub.points.length < 2) {
      report.dropped += 1
      continue
    }

    if (sub.closed) {
      const pts = toCCW(cleanPoints(sub.points, true, 1e-4))
      if (pts.length < 3) {
        report.dropped += 1
        continue
      }
      cands.push({ points: pts, closed: true, warnings: [] })
      continue
    }

    let pts = cleanPoints(sub.points, false, 1e-4)

    // 手绘导出的路径常无闭合标记但首尾几乎相接：在容差内吸附首尾并自动闭合，而不是丢弃
    if (pts.length >= 3 && dist(pts[0], pts[pts.length - 1]) <= opts.closeToleranceMm) {
      // 尾点吸附到首点（移动距离 ≤ 闭合容差），消除闭合错台，并与精确闭合的同形路径对齐
      const joined = [pts[0], ...pts.slice(1, -1), pts[0]]
      const closedPts = toCCW(cleanPoints(joined, true, 1e-4))
      if (closedPts.length >= 3) {
        report.autoClosed += 1
        cands.push({ points: closedPts, closed: true, warnings: ['auto_closed'] })
        continue
      }
      // 闭合后退化成点/线段（如一笔来回描）：无切割意义，丢弃
      report.dropped += 1
      continue
    }

    if (pts.length < 2) {
      report.dropped += 1
      continue
    }
    report.notClosed += 1
    cands.push({ points: pts, closed: false, warnings: ['not_closed'] })
  }

  // 重复路径合并：粗网格分桶（跨 20mm 桶边界的副本共享同一桶）+ 桶内两两比对。
  // 几何比对基于双向点到折线距离，容忍顶点数差异与反向；并用并查集传递合并，
  // 同一条线画几遍（3 遍、5 遍……）就全部合为一条，而不是只成对合并。
  const n = cands.length
  const parent = Array.from({ length: n }, (_, i) => i)
  const find = (x: number): number => {
    let r = x
    while (parent[r] !== r) r = parent[r]
    while (parent[x] !== r) {
      const next = parent[x]
      parent[x] = r
      x = next
    }
    return r
  }
  const union = (a: number, b: number): void => {
    parent[find(a)] = find(b)
  }

  const buckets = new Map<string, number[]>()
  cands.forEach((c, i) => {
    // 桶键只区分闭合/开放；顶点数不同的手绘副本也进同一桶，由几何比对裁决
    for (const bin of duplicateBins(c.points)) {
      const k = `${c.closed ? 'c' : 'o'}:${bin}`
      const arr = buckets.get(k)
      if (arr) arr.push(i)
      else buckets.set(k, [i])
    }
  })

  const samePath = (a: Cand, b: Cand): boolean => pathsOverlap(a.points, b.points, a.closed, DUPLICATE_MATCH_MM)

  // 廉价预筛：周长相差超过 1.5mm 的路径不可能重合，跳过距离比对
  const lenOf = new Map<number, number>()
  cands.forEach((c, i) => lenOf.set(i, polylineLength(c.points, c.closed)))
  const couldMatch = (i: number, j: number): boolean => Math.abs((lenOf.get(i) ?? 0) - (lenOf.get(j) ?? 0)) <= 1.5

  for (const arr of buckets.values()) {
    if (arr.length < 2) continue
    const uniq = [...new Set(arr)]
    for (let x = 0; x < uniq.length; x++) {
      for (let y = x + 1; y < uniq.length; y++) {
        if (find(uniq[x]) === find(uniq[y])) continue
        if (!couldMatch(uniq[x], uniq[y])) continue
        if (samePath(cands[uniq[x]], cands[uniq[y]])) union(uniq[x], uniq[y])
      }
    }
  }

  // 按代表收集等价类（保持原始顺序，每类最早出现的一条为保留者）
  const classes = new Map<number, number[]>()
  for (let i = 0; i < n; i++) {
    const root = find(i)
    const arr = classes.get(root)
    if (arr) arr.push(i)
    else classes.set(root, [i])
  }

  const contours: Contour[] = []
  for (const members of classes.values()) {
    const keepIdx = members[0]
    const c = cands[keepIdx]
    const removed = members.slice(1)
    const warnings = [...c.warnings]
    if (removed.length > 0 && !warnings.includes('duplicate')) warnings.push('duplicate')
    const contour = makeContour(c.points, c.closed, warnings)
    if (removed.length > 0) {
      contour.dupCount = removed.length
      report.duplicates += removed.length
      report.duplicateGroups.push({
        kept: contour,
        removed: removed.map((ri) => makeContour(cands[ri].points, cands[ri].closed, cands[ri].warnings)),
      })
    }
    contours.push(contour)
  }

  // 自交检测
  for (const c of contours) {
    const hits = findSelfIntersections(c.points, c.closed)
    if (hits.length > 0) {
      c.warnings.push('self_intersect')
      report.selfIntersect += 1
      report.selfIntersections[c.id] = hits.length
    }
  }

  report.kept = contours.length
  return { contours, report }
}

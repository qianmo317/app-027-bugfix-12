import type { Contour, ContourWarning, Pt } from './types'
import {
  cleanPoints,
  cyclicMatch,
  dist,
  duplicateKey,
  findSelfIntersections,
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

export function makeContour(
  points: Pt[],
  closed: boolean,
  warnings: ContourWarning[] = [],
  extra: { dupCount?: number; autoCloseGapMm?: number } = {},
): Contour {
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
    dupCount: extra.dupCount ?? 1,
    autoCloseGapMm: extra.autoCloseGapMm,
  }
}

/**
 * 路径清理：去重合点 / 共线点 → 闭合检查（近闭合自动闭合、未闭合打标）→
 * 重复路径合并（完全重叠或反向重叠，同轨迹 N 条全部合为 1 条）→ 自交检测
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

  type Cand = {
    points: Pt[]
    closed: boolean
    warnings: ContourWarning[]
    autoCloseGapMm?: number
  }
  const cands: Cand[] = []

  for (const sub of raw) {
    if (sub.points.length < 2) {
      report.dropped += 1
      continue
    }
    let closed = sub.closed
    let pts = cleanPoints(sub.points, closed, 1e-4)

    // 未打闭合标记、但首尾几乎相接（≥3 点）：自动闭合并记录缺口，不丢弃
    let autoCloseGapMm: number | undefined
    if (!closed && pts.length >= 3 && dist(pts[0], pts[pts.length - 1]) <= opts.closeToleranceMm) {
      autoCloseGapMm = dist(pts[0], pts[pts.length - 1])
      closed = true
      // 保留首尾两个端点（不能把末端吸附到首端，否则会丢掉最后一个拐角），
      // 仅置闭合：切刀沿首尾连线收口，缝隙 ≤ closeToleranceMm。
      pts = toCCW(cleanPoints(pts, true, 1e-4))
      if (pts.length < 3) {
        report.dropped += 1
        continue
      }
      report.autoClosed += 1
    } else if (closed) {
      pts = toCCW(pts)
    }

    if (closed && pts.length < 3) {
      report.dropped += 1
      continue
    }

    const warnings: ContourWarning[] = []
    if (!closed) {
      warnings.push('not_closed')
      report.notClosed += 1
    } else if (autoCloseGapMm !== undefined) {
      warnings.push('auto_closed')
    }
    cands.push({ points: pts, closed, warnings, autoCloseGapMm })
  }

  // 重复路径合并：先按几何指纹分组，再在组内与各组保留者做带容差的循环比对（含反向）。
  // 同一条轨迹画了 N 遍（N≥2）时，N-1 条全部并入保留者，只切一次。
  const groups = new Map<string, number[]>()
  cands.forEach((c, i) => {
    const k = duplicateKey(c.points, c.closed)
    const arr = groups.get(k)
    if (arr) arr.push(i)
    else groups.set(k, [i])
  })
  /** cand 索引 → 所在组保留者索引 */
  const dupOf = new Map<number, number>()
  /** 保留者索引 → 被合并者索引 */
  const members = new Map<number, number[]>()
  for (const arr of groups.values()) {
    if (arr.length < 2) continue
    const reps: number[] = []
    for (const idx of arr) {
      let owner = -1
      for (const r of reps) {
        if (cyclicMatch(cands[r].points, cands[idx].points, 0.02)) {
          owner = r
          break
        }
      }
      if (owner < 0) reps.push(idx)
      else {
        dupOf.set(idx, owner)
        const mem = members.get(owner)
        if (mem) mem.push(idx)
        else members.set(owner, [idx])
        report.duplicates += 1
      }
    }
  }

  const contours: Contour[] = []
  // 先造保留者，再造被合并者（报告引用用），保证报告对象齐全
  const keptContour = new Map<number, Contour>()
  const removedContour = new Map<number, Contour>()
  cands.forEach((c, i) => {
    if (dupOf.has(i)) {
      removedContour.set(i, makeContour(c.points, c.closed, c.warnings, { autoCloseGapMm: c.autoCloseGapMm }))
      return
    }
    const dupCount = 1 + (members.get(i)?.length ?? 0)
    const warnings = c.warnings.slice()
    if (dupCount > 1 && !warnings.includes('duplicate')) warnings.push('duplicate')
    const contour = makeContour(c.points, c.closed, warnings, {
      dupCount,
      autoCloseGapMm: c.autoCloseGapMm,
    })
    keptContour.set(i, contour)
    contours.push(contour)
  })
  for (const [owner, mem] of members) {
    const kept = keptContour.get(owner)
    if (!kept) continue
    const removed = mem.map((i) => removedContour.get(i)).filter((x): x is Contour => !!x)
    report.duplicateGroups.push({ kept, removed })
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

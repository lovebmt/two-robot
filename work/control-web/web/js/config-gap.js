export function gapClearance(targetMm, followerTofX, leaderRearMm) {
  if (![targetMm, followerTofX, leaderRearMm].every(Number.isFinite)) return null;
  return targetMm - followerTofX - leaderRearMm;
}

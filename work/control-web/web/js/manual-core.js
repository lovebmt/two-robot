export const MANUAL_COMMANDS = {
  fwd: { v: 80, w: 0, label: 'Đang tiến' },
  back: { v: -80, w: 0, label: 'Đang lùi' },
  left: { v: 0, w: 0.8, label: 'Đang quay trái' },
  right: { v: 0, w: -0.8, label: 'Đang quay phải' },
};

export function manualAvailability(mode, robot) {
  if (mode !== 'solo') return { enabled: false, status: 'Không khả dụng' };
  if (!robot) return { enabled: false, status: 'Chưa chọn xe' };
  if (robot.online === false) return { enabled: false, status: 'Xe offline' };
  if (!['IDLE', 'JOG'].includes(robot.state)) {
    return { enabled: false, status: `Không khả dụng · ${robot.state}` };
  }
  return { enabled: true, status: 'Sẵn sàng' };
}

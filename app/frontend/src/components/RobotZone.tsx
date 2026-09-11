import React from 'react';
import Robot3DViewer from './Robot3DViewer';
import JogControlPanel from './JogControlPanel';
import { API_BASE } from '../config';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
  connected?: boolean;
  tcp_speed_actual?: number[];   // 6 分量原始反馈: Vx/Vy/Vz (m/s) + 角速度 (rad/s)
  tcp_speed_mm_s?: number;       // 后端预计算的 |Vlin| 合速度 (mm/s)
  qd_actual?: number[];
  load?: number;
  error_status?: number;
  error_details?: string[];
  tool_vector_actual?: number[];
  hand_type?: number[];    // int8 x4: 手系配置
  tool_index?: number;     // 当前工具坐标系索引
  run_queued_cmd?: number; // 算法队列当前执行段序号
  velocity_ratio?: number;     // 1016 关节速度比例 (%)
  xyz_velocity_ratio?: number; // 1019 笛卡尔位置速度比例 (%)
  r_velocity_ratio?: number;   // 1020 笛卡尔姿态速度比例 (%)
  digital_outputs?: number[];  // 16路数字输出状态 [DO1..DO16] (0或1)
  digital_output_bits?: number;// 64位数字输出端子状态位掩码
  gripper?: {
    connected?: boolean;
    position_mm?: number;
    state?: string;
    force_n?: number;
    specs?: any;
  };
}

interface RobotZoneProps {
  robotState: RobotState;
  activeTemplate?: string | null;
  meshVersion?: number;
  pathsVersion?: number;
  pathState?: 'raw' | 'auto' | 'poi' | 'auto_poi';
}

const formatNum = (val?: number, decimals: number = 1, threshold: number = 0.05): string => {
  if (val === undefined || val === null || Math.abs(val) < threshold) {
    return (0).toFixed(decimals);
  }
  const formatted = val.toFixed(decimals);
  return formatted === '-0.0' || formatted === '-0.00' || formatted.startsWith('-0.0') && parseFloat(formatted) === 0
    ? (0).toFixed(decimals)
    : formatted;
};

// spd: raw 6-component TCP velocity from backend in m/s → convert to mm/s for display
// scalar: pre-computed |Vlin| in mm/s from backend
const formatTcpSpeed = (spd?: number[], scalar?: number): { mag: string; vec: string } => {
  // Convert all 6 components from m/s → mm/s
  const c = (spd || [0, 0, 0, 0, 0, 0]).map(v => (v || 0) * 1000);
  const [vx, vy, vz, rx, ry, rz] = [c[0], c[1], c[2], c[3], c[4], c[5]];
  const mag = scalar !== undefined
    ? formatNum(scalar, 1, 0.05)
    : formatNum(Math.sqrt(vx * vx + vy * vy + vz * vz), 1, 0.05);
  const linVec = `[${formatNum(vx, 2, 0.005)}, ${formatNum(vy, 2, 0.005)}, ${formatNum(vz, 2, 0.005)}]`;
  const rotVec = `[${formatNum(rx, 2, 0.005)}, ${formatNum(ry, 2, 0.005)}, ${formatNum(rz, 2, 0.005)}]`;
  return { mag, vec: `${linVec} | ${rotVec}` };
};

const formatQdSpeed = (qd?: number[]) => {
  if (!qd || qd.length === 0) return '0.0 °/s [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]';
  const absMax = Math.max(...qd.map(Math.abs));
  const maxStr = formatNum(absMax, 1, 0.05);
  const list = qd.slice(0, 6).map(v => formatNum(v, 1, 0.05)).join(', ');
  return `${maxStr} °/s [${list}]`;
};

const getDigitalOutputs = (state: RobotState): number[] => {
  if (Array.isArray(state.digital_outputs) && state.digital_outputs.length > 0) {
    const arr = [...state.digital_outputs];
    while (arr.length < 16) arr.push(0);
    return arr.slice(0, 16);
  }
  if (state.digital_output_bits !== undefined && state.digital_output_bits !== null) {
    const bits = Number(state.digital_output_bits);
    return Array.from({ length: 16 }, (_, i) => (bits >> i) & 1);
  }
  return Array(16).fill(0);
};

const RobotZone: React.FC<RobotZoneProps> = ({
  robotState,
  activeTemplate = null,
  meshVersion = 0,
  pathsVersion = 0,
  pathState = 'raw',
}) => {
  const [pendingDos, setPendingDos] = React.useState<Record<number, boolean>>({});

  const handleToggleDo = async (doIndex: number, currentVal: number) => {
    if (pendingDos[doIndex]) return;

    const isConnected = robotState.connected ?? (
      robotState.status !== undefined && (robotState.pose?.some(v => v !== 0) || robotState.joint?.some(v => v !== 0))
    );
    if (!isConnected) {
      console.warn(`[RobotZone] Cannot toggle DO ${doIndex}: Robot is not connected.`);
      return;
    }

    const targetStatus = currentVal === 1 ? 0 : 1;
    setPendingDos(prev => ({ ...prev, [doIndex]: true }));
    try {
      const res = await fetch(`${API_BASE}/api/robot/set_do`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          index: doIndex,
          status: targetStatus,
          immediate: true,
        }),
      });
      if (!res.ok) {
        const errorData = await res.json().catch(() => ({ detail: 'Unknown error' }));
        console.error(`[RobotZone] Failed to set DO ${doIndex}:`, errorData.detail);
        alert(`Failed to set DO ${doIndex}: ${errorData.detail || res.statusText}`);
      }
    } catch (err: any) {
      console.error(`[RobotZone] Error toggling DO ${doIndex}:`, err);
      alert(`Error toggling DO ${doIndex}: ${err.message || 'Network error'}`);
    } finally {
      setPendingDos(prev => {
        const next = { ...prev };
        delete next[doIndex];
        return next;
      });
    }
  };

  const handleClearError = async () => {
    try {
      const res = await fetch(`${API_BASE}/api/robot/clear_error`, { method: 'POST' });
      if (!res.ok) {
        const error = await res.json();
        alert(`Clear Error failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      alert(`Clear Error error: ${err.message}`);
      console.error('Failed to clear error:', err);
    }
  };

  return (
    <div className="w-full h-full flex flex-col gap-3">
      {/* Robot 3D Viewer with Surface Mesh Overlay & 3D TCP Trajectories */}
      <div className="flex-1 min-h-0 bg-slate-900/80 rounded-xl border border-slate-800 shadow-lg flex flex-col shrink-0 p-1 relative overflow-hidden">
        <Robot3DViewer
          jointAngles={robotState.joint ?? [0, 0, 0, 0, 0, 0]}
          gripperStroke={robotState.gripper?.position_mm}
          activeTemplate={activeTemplate}
          meshVersion={meshVersion}
          pathsVersion={pathsVersion}
          pathState={pathState}
        />

        {/* Centered Error Overlay */}
        {!!robotState.error_status && (
          <div className="absolute inset-0 flex items-center justify-center z-50 pointer-events-none">
            <div className="bg-red-950/80 backdrop-blur-md border border-red-500/50 p-2.5 rounded-lg flex items-center gap-4 shadow-[0_0_20px_rgba(239,68,68,0.3)] pointer-events-auto transition-all transform animate-in fade-in zoom-in-95 duration-200 min-w-[450px] max-w-[90%]">
              
              {/* Left side: Icon & Code */}
              <div className="flex flex-col items-center gap-1 border-r border-red-500/30 pr-4 shrink-0">
                <div className="w-8 h-8 bg-red-500/20 rounded-full flex items-center justify-center">
                  <span className="text-red-500 text-lg animate-pulse">⚠️</span>
                </div>
                <div className="flex flex-col items-center text-center">
                  <h3 className="text-red-400 font-bold text-[11px] tracking-wide uppercase">Robot Alarm</h3>
                  <p className="text-red-300/80 text-[10px] font-mono leading-none">Code: <strong className="text-white">{robotState.error_status}</strong></p>
                </div>
              </div>

              {/* Middle side: Error Details */}
              <div className="flex-1 flex flex-col gap-1 max-h-[80px] overflow-y-auto hide-scrollbar">
                {robotState.error_details && robotState.error_details.length > 0 ? (
                  robotState.error_details.map((msg, idx) => (
                    <div key={idx} className="text-red-200 text-[11px] font-medium bg-red-900/40 px-2 py-1 rounded-sm border border-red-500/20 break-words leading-tight">
                      {msg}
                    </div>
                  ))
                ) : (
                  <div className="text-red-300/60 text-[11px] italic">Unknown error details (Check logs)</div>
                )}
              </div>

              {/* Right side: Button */}
              <button
                onClick={handleClearError}
                className="shrink-0 px-3 py-1.5 bg-red-600 hover:bg-red-500 active:scale-95 text-white rounded-md text-[11px] font-bold shadow-[0_0_10px_rgba(239,68,68,0.4)] transition-all cursor-pointer whitespace-nowrap"
              >
                CLEAR ERROR
              </button>
            </div>
          </div>
        )}

        {/* Live Robot Speed HUD (Bottom-Left Ultra-Transparent HUD Overlay) */}
        <div className="absolute bottom-2.5 left-2.5 z-10 pointer-events-none flex flex-col gap-0.5 px-2 py-1 font-mono text-[10px] select-none drop-shadow-[0_1px_3px_rgba(0,0,0,0.95)]">
          {/* TCP Speed Row */}
          {(() => {
            const { mag, vec } = formatTcpSpeed(robotState.tcp_speed_actual, robotState.tcp_speed_mm_s);
            return (
              <>
                <div className="flex items-center gap-1.5">
                  <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">TCP SPEED:</span>
                  <span className="text-sky-300 font-bold tracking-tight">{mag} mm/s</span>
                </div>
                <div className="flex items-center gap-1.5 ml-1">
                  <span className="text-slate-400/70 text-[8px] uppercase tracking-wider shrink-0">XYZ|RPY:</span>
                  <span className="text-sky-200/80 tracking-tight text-[8.5px]">{vec}</span>
                </div>
              </>
            );
          })()}
          {/* Joint Speed Row */}
          <div className="flex items-center gap-1.5">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">JNT SPEED:</span>
            <span className="text-emerald-300 font-bold tracking-tight">
              {formatQdSpeed(robotState.qd_actual)}
            </span>
          </div>
          {/* Hand Type & Tool Index Row */}
          <div className="flex items-center gap-2">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">TOOL:</span>
            <span className="text-amber-300 font-bold tracking-tight">
              #{robotState.tool_index ?? 0}
            </span>
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0 ml-1">HAND:</span>
            <span className="text-amber-200/80 tracking-tight text-[8.5px]">
              [{(robotState.hand_type ?? [0,0,0,0]).join(', ')}]
            </span>
          </div>
          {/* Speed Ratio Row: J% / XYZ% / R% */}
          <div className="flex items-center gap-1.5">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">SPD%:</span>
            <span className="text-lime-300 font-bold tracking-tight">
              J:{robotState.velocity_ratio ?? 0}%
            </span>
            <span className="text-slate-500/80 text-[8px]">|</span>
            <span className="text-lime-200/80 tracking-tight text-[8.5px]">
              XYZ:{robotState.xyz_velocity_ratio ?? 0}%
            </span>
            <span className="text-slate-500/80 text-[8px]">|</span>
            <span className="text-lime-200/80 tracking-tight text-[8.5px]">
              R:{robotState.r_velocity_ratio ?? 0}%
            </span>
          </div>
          {/* Queue Progress Row — only shown when robot is moving */}
          {(robotState.status === 1) && (
            <div className="flex items-center gap-1.5">
              <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">QUEUE SEG:</span>
              <span className="text-violet-300 font-bold tracking-tight">
                {robotState.run_queued_cmd ?? 0}
              </span>
            </div>
          )}
          {/* DO (Digital Output 1~16) Interactive Circular Row */}
          {(() => {
            const dos = getDigitalOutputs(robotState);
            const isConnected = robotState.connected ?? (
              robotState.status !== undefined && (robotState.pose?.some(v => v !== 0) || robotState.joint?.some(v => v !== 0))
            );
            return (
              <div className="flex items-center gap-1.5 mt-0.5 pointer-events-auto select-none">
                <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">
                  DO:
                </span>
                <div className="flex items-center gap-1">
                  {dos.map((val, idx) => {
                    const doIndex = idx + 1;
                    const isOn = val === 1;
                    const isPending = !!pendingDos[doIndex];
                    return (
                      <button
                        key={idx}
                        type="button"
                        onClick={() => handleToggleDo(doIndex, val)}
                        disabled={!isConnected || isPending}
                        title={
                          !isConnected
                            ? `DO ${doIndex}: ${isOn ? 'ON (1)' : 'OFF (0)'} (Robot offline)`
                            : isPending
                            ? `DO ${doIndex}: Switching...`
                            : `DO ${doIndex}: ${isOn ? 'ON (1) - Click to turn OFF' : 'OFF (0) - Click to turn ON'}`
                        }
                        className={`w-[15px] h-[15px] min-w-[15px] text-[8px] flex items-center justify-center rounded-full font-mono leading-none font-bold transition-all duration-150 ${
                          isOn
                            ? 'bg-emerald-500/30 text-emerald-300 border border-emerald-400/80 shadow-[0_0_6px_rgba(16,185,129,0.5)]'
                            : 'bg-slate-900/70 text-slate-400/70 border border-slate-700/50'
                        } ${
                          isConnected
                            ? 'cursor-pointer hover:scale-115 hover:border-sky-400 hover:text-white active:scale-90'
                            : 'cursor-not-allowed opacity-50'
                        } ${
                          isPending ? 'animate-pulse ring-1 ring-sky-400 border-sky-400' : ''
                        } ${idx === 7 ? 'mr-1.5' : ''}`}
                      >
                        {doIndex}
                      </button>
                    );
                  })}
                </div>
              </div>
            );
          })()}
        </div>
      </div>

      {/* Jog Control Panel & Connection */}
      <div className="shrink-0">
        <JogControlPanel robotState={robotState} />
      </div>
    </div>
  );
};

export default RobotZone;

import React, { useEffect, useState, useRef } from 'react';
import { Cpu, Crosshair, Home, Pause, Play, AlertOctagon, Minimize2, ChevronDown, ChevronUp, Hand } from 'lucide-react';
import { API_BASE, WS_BASE } from '../config';

interface GripperSpecs {
  model?: string;
  total_stroke_mm?: number;
  single_finger_stroke_mm?: number;
  min_stroke_mm?: number;
  max_stroke_mm?: number;
  default_stroke_mm?: number;
  max_force_n?: number;
  min_force_n?: number;
  default_force_percent?: number;
  default_speed_percent?: number;
  open_speed_percent?: number;
  clamp_speed_percent?: number;
  full_stroke_time_s?: number;
}

interface RobotState {
  pose?: number[];
  joint?: number[];
  status?: number;
  connected?: boolean;
  tcp_speed_actual?: number[];
  qd_actual?: number[];
  load?: number;
  error_status?: number;
  tool_vector_actual?: number[];
  digital_outputs?: number[];
  digital_output_bits?: number;
  gripper?: {
    connected?: boolean;
    position_mm?: number;
    state?: string;
    force_n?: number;
    specs?: GripperSpecs;
  };
}

interface JogControlPanelProps {
  robotState?: RobotState;
}

const JogControlPanel: React.FC<JogControlPanelProps> = ({ robotState }) => {
  const [speedL, setSpeedL] = useState<number>(100);
  const [accL, setAccL] = useState<number>(20);
  const [speedJ, setSpeedJ] = useState<number>(20);
  const [accJ, setAccJ] = useState<number>(20);

  const [globalSpeedFactor, setGlobalSpeedFactor] = useState<number>(50);
  const [maxTcpSpeed, setMaxTcpSpeed] = useState<number>(2000);
  const [maxJointSpeed, setMaxJointSpeed] = useState<number>(180);

  // 根据最大值和全局比例计算具体可用区间上限
  const effMaxLinSpeed = Math.max(10, Math.round(maxTcpSpeed * (globalSpeedFactor / 100.0)));
  const effMaxJntSpeed = Math.max(5, Math.round(maxJointSpeed * (globalSpeedFactor / 100.0)));

  // Use prop robotState if provided, ensuring pose and joint arrays are never undefined
  const displayState: RobotState = {
    ...robotState,
    pose: robotState?.pose ?? [0, 0, 0, 0, 0, 0],
    joint: robotState?.joint ?? [0, 0, 0, 0, 0, 0],
  };
  const [robotConnected, setRobotConnected] = useState<boolean>(robotState?.connected ?? false);
  const [connecting, setConnecting] = useState<boolean>(false);
  const [activeAction, setActiveAction] = useState<'home' | 'zero' | 'fold' | null>(null);

  // Track active jog button
  const [activeJog, setActiveJog] = useState<{ axis: string, dir: number } | null>(null);

  // Panel collapse toggle: fold everything below the toolbar to maximize the 3D viewer
  const [collapsed, setCollapsed] = useState<boolean>(false);

  // Gripper specs & state (specs sourced from JunduoGripper driver)
  const gripperSpecs = displayState.gripper?.specs;
  const maxStrokeMm = gripperSpecs?.total_stroke_mm ?? 50.0;
  const maxForceN = gripperSpecs?.max_force_n ?? 60.0;

  // Gripper state (defaults to closed: 0.0mm)
  const [targetStroke, setTargetStroke] = useState<number>(0.0);
  const [gripperSpeed, setGripperSpeed] = useState<number>(gripperSpecs?.default_speed_percent ?? 50);
  const [gripperForce, setGripperForce] = useState<number>(gripperSpecs?.default_force_percent ?? 50);
  const [gripperOperating, setGripperOperating] = useState<boolean>(false);

  const isMoving = displayState.status === 1;
  const disableMotion = !robotConnected || isMoving;

  // Connection and interlock checks: the gripper is operable whenever the device
  // is connected and not currently actuating — independent of robot motion state
  const isGripperConnected = !!(displayState.gripper?.connected && robotConnected);
  const disableGripper = !isGripperConnected || gripperOperating;

  // Speed dynamics are read-only when the robot is offline (grayed out like the gripper widget)
  const disableSpeed = !robotConnected;

  // Toolbar gripper quick toggle: derive open/closed from live telemetry with a
  // mid-stroke threshold (>50% stroke = open), so the button state stays consistent
  // with the panel CLAMP/OPEN actions and any external gripper motion via WebSocket.
  const gripperIsOpen = isGripperConnected && (displayState.gripper?.position_mm ?? 0) > maxStrokeMm * 0.5;

  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (robotState?.connected !== undefined) {
      setRobotConnected(robotState.connected);
    }
  }, [robotState?.connected]);

  useEffect(() => {
    if (!isMoving) {
      setActiveAction(null);
    }
  }, [isMoving]);

  // Synchronize targetStroke with backend reported position when not dragging
  useEffect(() => {
    if (displayState.gripper?.position_mm !== undefined && !gripperOperating) {
      setTargetStroke(displayState.gripper.position_mm);
    }
  }, [displayState.gripper?.position_mm, gripperOperating]);

  const handleGripperMove = async (stroke: number) => {
    if (!isGripperConnected) return;
    setGripperOperating(true);
    try {
      await fetch(`${API_BASE}/api/robot/gripper/move`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ stroke_mm: stroke, force_percent: gripperForce, speed: gripperSpeed })
      });
    } catch (err) {
      console.error('Failed to move gripper:', err);
    } finally {
      setTimeout(() => setGripperOperating(false), 300);
    }
  };

  const handleGripperOpen = async () => {
    if (!isGripperConnected) return;
    setTargetStroke(maxStrokeMm);
    setGripperOperating(true);
    try {
      await fetch(`${API_BASE}/api/robot/gripper/open`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ force_percent: gripperForce, speed: gripperSpeed })
      });
    } catch (err) {
      console.error('Failed to open gripper:', err);
    } finally {
      setTimeout(() => setGripperOperating(false), 300);
    }
  };

  const handleGripperClamp = async () => {
    if (!isGripperConnected) return;
    setTargetStroke(0.0);
    setGripperOperating(true);
    try {
      await fetch(`${API_BASE}/api/robot/gripper/clamp`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ force_percent: gripperForce, speed: gripperSpeed })
      });
    } catch (err) {
      console.error('Failed to clamp gripper:', err);
    } finally {
      setTimeout(() => setGripperOperating(false), 300);
    }
  };

  useEffect(() => {
    fetchSpeed();
    connectWs();
    return () => {
      if (wsRef.current) wsRef.current.close();
    };
  }, []);

  const connectWs = () => {
    const ws = new WebSocket(`${WS_BASE}/api/robot/ws`);
    ws.onopen = () => { };
    ws.onclose = () => {
      setTimeout(connectWs, 2000); // Reconnect WebSocket
    };
    ws.onmessage = (_e) => {
      // Robot state is now handled at CalibView level and passed as a prop
    };
    wsRef.current = ws;
  };

  const fetchSpeed = async () => {
    try {
      const spdRes = await fetch(`${API_BASE}/api/robot/speed`);
      if (spdRes.ok) {
        const spdData = await spdRes.json();
        const gFactor = spdData.global_speed_factor ?? 50;
        const maxTcp = spdData.max_tcp_speed_mm_s ?? 2000;
        const maxJnt = Array.isArray(spdData.max_joint_speed_deg_s)
          ? spdData.max_joint_speed_deg_s[0]
          : (spdData.max_joint_speed_deg_s ?? 180);

        setGlobalSpeedFactor(gFactor);
        setMaxTcpSpeed(maxTcp);
        setMaxJointSpeed(maxJnt);

        const currentEffMaxLin = Math.max(10, Math.round(maxTcp * (gFactor / 100.0)));
        const currentEffMaxJnt = Math.max(5, Math.round(maxJnt * (gFactor / 100.0)));

        if (spdData.speed_l !== undefined && spdData.speed_l !== null) {
          setSpeedL(Math.min(currentEffMaxLin, Math.max(1, Math.round(spdData.speed_l))));
        } else {
          setSpeedL(Math.min(currentEffMaxLin, 100));
        }

        if (spdData.speed_j !== undefined && spdData.speed_j !== null) {
          setSpeedJ(Math.min(currentEffMaxJnt, Math.max(1, Math.round(spdData.speed_j))));
        } else {
          setSpeedJ(Math.min(currentEffMaxJnt, 20));
        }

        setAccL(spdData.acc_l || 20);
        setAccJ(spdData.acc_j || 20);
      }
    } catch (e) {
      console.error("Failed to fetch initial speed", e);
    }
  };

  const handleConnect = async () => {
    if (connecting) return;
    setConnecting(true);

    if (robotConnected) {
      // Disconnect
      try {
        const res = await fetch(`${API_BASE}/api/robot/disconnect`, { method: 'POST' });
        if (res.ok) {
          setRobotConnected(false);
        }
      } catch (err) {
        console.error('Disconnect error:', err);
      }
    } else {
      // Connect
      try {
        const res = await fetch(`${API_BASE}/api/robot/connect`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ robot_type: 'dobot' })
        });
        if (res.ok) {
          setRobotConnected(true);
          // Fetch current global speed
          fetchSpeed();
        } else {
          console.error('Failed to connect to robot.');
        }
      } catch (err: any) {
        console.error('Connect error:', err);
      }
    }
    setConnecting(false);
  };

  const handleSpeedUpdate = async () => {
    try {
      await fetch(`${API_BASE}/api/robot/speed`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed_l: speedL, acc_l: accL, speed_j: speedJ, acc_j: accJ })
      });
    } catch (err) {
      console.error('Failed to sync speed:', err);
    }
  };

  const syncGlobalSpeed = async (newFactor: number) => {
    try {
      await fetch(`${API_BASE}/api/robot/global_speed`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ factor: newFactor })
      });
      fetchSpeed(); // Re-fetch to update effective caps
    } catch (err) {
      console.error('Failed to sync global speed factor:', err);
    }
  };

  const handleJogContinuous = async (axis: string, direction: number) => {
    if (direction !== 0) {
      setActiveJog({ axis, dir: direction });
    } else {
      setActiveJog(null);
    }
    try {
      const res = await fetch(`${API_BASE}/api/robot/jog_continuous`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ axis, direction })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Jog failed: ${error.detail || 'Unknown error'}`);
        setActiveJog(null);
      }
    } catch (err: any) {
      console.error(`Jog error: ${err.message}`);
      setActiveJog(null);
    }
  };

  const handleZero = async () => {
    setActiveAction('zero');
    try {
      const res = await fetch(`${API_BASE}/api/robot/zero`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Zero failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      console.error('Failed to go zero:', err);
    } finally {
      setActiveAction(null);
    }
  };

  const handleHome = async () => {
    setActiveAction('home');
    try {
      const res = await fetch(`${API_BASE}/api/robot/home`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Home failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      console.error('Failed to go home:', err);
    } finally {
      setActiveAction(null);
    }
  };

  const handleFold = async () => {
    setActiveAction('fold');
    try {
      const res = await fetch(`${API_BASE}/api/robot/fold`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Fold failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      console.error('Failed to fold robot:', err);
    } finally {
      setActiveAction(null);
    }
  };

  const handlePause = async () => {
    try {
      await fetch(`${API_BASE}/api/robot/pause`, { method: 'POST' });
    } catch (err: any) { console.error('Pause error:', err); }
  };

  const handleResume = async () => {
    try {
      await fetch(`${API_BASE}/api/robot/resume`, { method: 'POST' });
    } catch (err: any) { console.error('Resume error:', err); }
  };

  const handleEstop = async () => {
    try {
      await fetch(`${API_BASE}/api/robot/estop`, { method: 'POST' });
    } catch (err: any) {
      alert(`E-Stop triggered!`);
      console.error('E-Stop error:', err);
    }
  };

  const cartesianAxes = [
    { name: 'X', unit: 'mm' }, { name: 'Y', unit: 'mm' }, { name: 'Z', unit: 'mm' },
    { name: 'Rx', unit: '°' }, { name: 'Ry', unit: '°' }, { name: 'Rz', unit: '°' }
  ];

  const jointAxes = [
    { name: 'J1', unit: '°' }, { name: 'J2', unit: '°' }, { name: 'J3', unit: '°' },
    { name: 'J4', unit: '°' }, { name: 'J5', unit: '°' }, { name: 'J6', unit: '°' }
  ];

  const renderAxisRow = (axis: { name: string, unit: string }, idx: number, isJoint: boolean) => {
    const currentValues = (isJoint ? displayState.joint : displayState.pose) ?? [0, 0, 0, 0, 0, 0];

    const formatValue = (idx: number, isJoint: boolean) => {
      let val = currentValues[idx];
      if (val === undefined || val === null) return '0.00';
      if (!isJoint && idx >= 3) {
        val = val * (180 / Math.PI);
      }
      if (Math.abs(val) < 0.005) return '0.00';
      const formatted = val.toFixed(2);
      return formatted === '-0.00' || (formatted.startsWith('-0.0') && parseFloat(formatted) === 0) ? '0.00' : formatted;
    };

    let pulseDuration = '0s';
    const isJoggingNeg = activeJog?.axis === axis.name && activeJog?.dir === -1;
    const isJoggingPos = activeJog?.axis === axis.name && activeJog?.dir === 1;

    if (activeJog && (isJoggingNeg || isJoggingPos)) {
      let speed = 10;
      if (isJoint) {
        let v = (displayState.qd_actual && displayState.qd_actual[idx]) || 0;
        speed = Math.abs(v);
      } else {
        if (displayState.tcp_speed_actual && displayState.tcp_speed_actual.length > idx) {
          let v = displayState.tcp_speed_actual[idx];
          if (idx < 3) {
            // backend tcp_speed_actual is m/s, convert to mm/s
            speed = Math.abs(v) * 1000;
          } else {
            // rad/s to deg/s
            speed = Math.abs(v) * (180 / Math.PI);
          }
        } else {
          speed = speedL;
        }
      }

      // Calculate duration inversely proportional to speed
      // Map 10mm/s -> 1s, 50mm/s -> 0.2s
      speed = Math.max(0.1, speed);
      let d = 10 / speed;
      d = Math.min(1.5, Math.max(0.1, d)); // Clamp between 0.1s and 1.5s
      pulseDuration = d.toFixed(3) + 's';
    }

    const getBtnClass = (isActive: boolean) => `
      w-8 h-7 flex items-center justify-center border rounded transition-colors select-none text-[14px] font-bold
      ${isActive
        ? 'bg-blue-600 border-blue-400 text-white animate-pulse shadow-[0_0_12px_rgba(59,130,246,0.9)] z-10'
        : 'bg-slate-800 border-slate-700 text-slate-300 hover:bg-slate-700 hover:text-white'}
      ${disableMotion && !isActive ? 'opacity-50 cursor-not-allowed hover:bg-slate-800' : ''}
    `;

    return (
      <div key={axis.name} className="flex items-center gap-1">
        <button
          onPointerDown={(e) => { e.currentTarget.releasePointerCapture(e.pointerId); handleJogContinuous(axis.name, -1); }}
          onPointerUp={() => handleJogContinuous(axis.name, 0)}
          onPointerLeave={() => { if (activeJog?.axis === axis.name && activeJog?.dir === -1) handleJogContinuous(axis.name, 0); }}
          onContextMenu={(e) => e.preventDefault()}
          disabled={disableMotion && !isJoggingNeg}
          className={getBtnClass(isJoggingNeg)}
          style={isJoggingNeg ? { animationDuration: pulseDuration } : {}}
        >
          -
        </button>

        <div className="flex-1 flex items-center justify-between bg-slate-950 border border-slate-800 rounded px-1.5 h-7">
          <span className="text-[10px] font-bold text-slate-500 w-[18px]">{axis.name}</span>
          <span className="text-[11px] font-mono text-emerald-400 tracking-wider">
            {formatValue(idx, isJoint)}
          </span>
          <span className="text-[8px] text-slate-600 w-[10px]">{axis.unit}</span>
        </div>

        <button
          onPointerDown={(e) => { e.currentTarget.releasePointerCapture(e.pointerId); handleJogContinuous(axis.name, 1); }}
          onPointerUp={() => handleJogContinuous(axis.name, 0)}
          onPointerLeave={() => { if (activeJog?.axis === axis.name && activeJog?.dir === 1) handleJogContinuous(axis.name, 0); }}
          onContextMenu={(e) => e.preventDefault()}
          disabled={disableMotion && !isJoggingPos}
          className={getBtnClass(isJoggingPos)}
          style={isJoggingPos ? { animationDuration: pulseDuration } : {}}
        >
          +
        </button>
      </div>
    );
  };

  return (
    <div className="bg-slate-900/80 rounded-xl border border-slate-800 p-2.5 shadow-lg backdrop-blur-sm shrink-0 w-full flex flex-col gap-2 relative">
      {/* Toolbar Row: Status LED + Action Buttons + Panel Collapse Toggle (always visible, also the collapsed header) */}
      <div className="flex items-center gap-2">
        {/* Status LED Indicator */}
        <div className="flex items-center gap-1 shrink-0 pointer-events-none">
          <div className={`w-2 h-2 rounded-full transition-colors duration-300 ${!robotConnected ? 'bg-slate-600' : isMoving ? 'bg-amber-400 animate-pulse shadow-[0_0_8px_rgba(251,191,36,0.6)]' : 'bg-emerald-500 shadow-[0_0_8px_rgba(16,185,129,0.6)]'}`}></div>
          <span className="text-[9px] font-medium uppercase tracking-wider text-slate-400">
            {!robotConnected ? 'Offline' : isMoving ? 'Moving' : 'Idle'}
          </span>
        </div>
        {/* Action Buttons (Single Row, compact & evenly distributed) */}
        <div className="flex flex-row gap-0.5 flex-1 justify-between min-w-0">
          <button
            onClick={handleConnect}
            disabled={connecting}
            className={`px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border disabled:opacity-50 ${robotConnected ? 'bg-green-600/20 text-green-500 hover:bg-green-600 hover:text-white border-green-600/30' : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50'}`}
          >
            <Cpu size={12} className="shrink-0" /> <span>{connecting ? '...' : robotConnected ? 'Disconnect' : 'Connect'}</span>
          </button>
          <button
            onClick={handleHome}
            disabled={disableMotion}
            className={`px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${activeAction === 'home'
                ? 'bg-emerald-600/20 text-emerald-400 border-emerald-500/50 animate-pulse shadow-[0_0_8px_rgba(16,185,129,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
              }`}
          >
            <Home size={12} className="shrink-0" /> <span>Home</span>
          </button>
          <button
            onClick={handleZero}
            disabled={disableMotion}
            className={`px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${activeAction === 'zero'
                ? 'bg-emerald-600/20 text-emerald-400 border-emerald-500/50 animate-pulse shadow-[0_0_8px_rgba(16,185,129,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
              }`}
          >
            <Crosshair size={12} className="shrink-0" /> <span>Zero</span>
          </button>
          <button
            onClick={handleFold}
            disabled={disableMotion}
            title="Move robot to configured Fold position"
            className={`px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${activeAction === 'fold'
                ? 'bg-purple-600/20 text-purple-400 border-purple-500/50 animate-pulse shadow-[0_0_8px_rgba(168,85,247,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
              }`}
          >
            <Minimize2 size={12} className="shrink-0" /> <span>Fold</span>
          </button>
          <button
            onClick={handlePause}
            disabled={!robotConnected}
            className="px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border border-slate-700/50 disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <Pause size={12} className="shrink-0" /> <span>Pause</span>
          </button>
          <button
            onClick={handleResume}
            disabled={!robotConnected}
            className="px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border border-slate-700/50 disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <Play size={12} className="shrink-0" /> <span>Resume</span>
          </button>
          {/* Gripper Quick Toggle: open when closed / close when open; state synced from live gripper telemetry */}
          <button
            onClick={gripperIsOpen ? handleGripperClamp : handleGripperOpen}
            disabled={disableGripper}
            title="Toggle gripper open/close (state synced from live gripper telemetry)"
            className={`px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${
              gripperOperating
                ? 'bg-amber-600/20 text-amber-400 border-amber-500/50 animate-pulse cursor-not-allowed'
                : gripperIsOpen
                ? 'bg-amber-600/20 text-amber-300 hover:bg-amber-600 hover:text-white border-amber-600/30'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
            }`}
          >
            <Hand size={12} className="shrink-0" /> <span>{gripperIsOpen ? 'Close' : 'Open'}</span>
          </button>
          <button
            onClick={handleEstop}
            disabled={!robotConnected}
            className="px-1.5 h-7 text-[11px] whitespace-nowrap rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors bg-red-600/20 text-red-500 hover:bg-red-600 hover:text-white border border-red-600/30 disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <AlertOctagon size={12} className="shrink-0" /> <span>Stop</span>
          </button>
        </div>

        {/* Panel Collapse Toggle: fold the body to maximize the 3D viewer area */}
        <button
          onClick={() => setCollapsed((c) => !c)}
          title={collapsed ? 'Expand control panel' : 'Collapse control panel'}
          className="shrink-0 w-6 h-6 flex items-center justify-center rounded-md bg-slate-800/80 border border-slate-700/50 text-slate-400 hover:text-white hover:bg-slate-700 transition-colors"
        >
          {collapsed ? <ChevronUp size={13} /> : <ChevronDown size={13} />}
        </button>
      </div>

      {/* Three-Column Body: Cartesian Axes | Joint Axes | Speed Dynamics + Gripper */}
      {!collapsed && (
        <div className="flex gap-2.5 min-h-0">
          {/* Cartesian (Left) — jog layout preserved, fixed narrow width */}
          <div className="flex flex-col gap-1.5 w-[176px] shrink-0">
            {cartesianAxes.map((axis, idx) => renderAxisRow(axis, idx, false))}
          </div>

          {/* Divider */}
          <div className="w-[1px] bg-slate-800/60 rounded-full" />

          {/* Joint (Middle) — jog layout preserved, fixed narrow width */}
          <div className="flex flex-col gap-1.5 w-[176px] shrink-0">
            {jointAxes.map((axis, idx) => renderAxisRow(axis, idx, true))}
          </div>

          {/* Divider */}
          <div className="w-[1px] bg-slate-800/60 rounded-full" />

          {/* Right: Global Speed + Speed/Accel + Gripper (Vertical Stack, takes remaining width) */}
          <div className="flex-1 min-w-0 flex flex-col gap-1.5">
            {/* Global Speed & Effective Caps */}
            <div className="flex flex-col gap-1 bg-slate-950/40 p-1.5 rounded-lg border border-slate-800/40">
              <div className="flex items-center gap-1.5">
                <span className="tracking-wider uppercase text-slate-400 font-semibold text-[9px] shrink-0">Global</span>
                <input
                  type="range"
                  disabled={disableSpeed}
                  className={`slider-compact flex-1 h-1 bg-slate-700/50 rounded-lg appearance-none cursor-pointer text-amber-500 ${
                    disableSpeed ? 'opacity-30 cursor-not-allowed' : ''
                  }`}
                  min="1" max="100" step="1"
                  value={globalSpeedFactor}
                  onChange={(e) => setGlobalSpeedFactor(parseInt(e.target.value))}
                  onMouseUp={(e) => syncGlobalSpeed(parseInt((e.target as HTMLInputElement).value))}
                  onTouchEnd={(e) => syncGlobalSpeed(parseInt((e.target as HTMLInputElement).value))}
                />
                <span className={`font-mono font-semibold text-[9px] w-7 text-right shrink-0 ${disableSpeed ? 'text-slate-500' : 'text-amber-400'}`}>{globalSpeedFactor}%</span>
              </div>
              {/* Effective caps (mm/s, °/s) & payload — gray when robot offline */}
              <div className="font-mono text-slate-400 flex flex-wrap items-center gap-x-1 gap-y-0.5 text-[8.5px]">
                <span>TCP <strong className={disableSpeed ? 'text-slate-500' : 'text-sky-400'}>{effMaxLinSpeed} mm/s</strong></span>
                <span className="text-slate-600">|</span>
                <span>JNT <strong className={disableSpeed ? 'text-slate-500' : 'text-emerald-400'}>{effMaxJntSpeed} °/s</strong></span>
                <span className="text-slate-600">|</span>
                <span>Load <strong className={disableSpeed ? 'text-slate-500' : 'text-purple-400'}>{(displayState.load ?? 0).toFixed(2)} kg</strong></span>
              </div>
            </div>

            {/* Linear / Joint Speed & Accel — 2x2 compact grid (gray text when robot offline) */}
            <div className="grid grid-cols-2 gap-x-2 gap-y-1.5">
              <div className="flex flex-col gap-0.5 min-w-0">
                <div className="flex justify-between items-baseline text-[9px] font-medium">
                  <span className="text-slate-400">LIN SPEED</span>
                  <span className={`font-mono font-semibold ${disableSpeed ? 'text-slate-500' : 'text-blue-400'}`}>{speedL} mm/s</span>
                </div>
                <input
                  type="range"
                  disabled={disableSpeed}
                  className={`slider-compact w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer text-blue-500 ${
                    disableSpeed ? 'cursor-not-allowed' : ''
                  }`}
                  min="1"
                  max={effMaxLinSpeed}
                  step="1"
                  value={Math.min(speedL, effMaxLinSpeed)}
                  onChange={(e) => setSpeedL(Number(e.target.value))}
                  onPointerUp={handleSpeedUpdate}
                  onKeyUp={handleSpeedUpdate}
                />
              </div>
              <div className="flex flex-col gap-0.5 min-w-0">
                <div className="flex justify-between items-baseline text-[9px] font-medium">
                  <span className="text-slate-400">LIN ACCEL</span>
                  <span className={`font-mono font-semibold ${disableSpeed ? 'text-slate-500' : 'text-blue-400'}`}>{accL}%</span>
                </div>
                <input
                  type="range"
                  disabled={disableSpeed}
                  className={`slider-compact w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer text-blue-500 ${
                    disableSpeed ? 'cursor-not-allowed' : ''
                  }`}
                  min="1"
                  max="100"
                  value={accL}
                  onChange={(e) => setAccL(Number(e.target.value))}
                  onPointerUp={handleSpeedUpdate}
                  onKeyUp={handleSpeedUpdate}
                />
              </div>
              <div className="flex flex-col gap-0.5 min-w-0">
                <div className="flex justify-between items-baseline text-[9px] font-medium">
                  <span className="text-slate-400">JNT SPEED</span>
                  <span className={`font-mono font-semibold ${disableSpeed ? 'text-slate-500' : 'text-green-400'}`}>{speedJ} °/s</span>
                </div>
                <input
                  type="range"
                  disabled={disableSpeed}
                  className={`slider-compact w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer text-green-500 ${
                    disableSpeed ? 'cursor-not-allowed' : ''
                  }`}
                  min="1"
                  max={effMaxJntSpeed}
                  step="1"
                  value={Math.min(speedJ, effMaxJntSpeed)}
                  onChange={(e) => setSpeedJ(Number(e.target.value))}
                  onPointerUp={handleSpeedUpdate}
                  onKeyUp={handleSpeedUpdate}
                />
              </div>
              <div className="flex flex-col gap-0.5 min-w-0">
                <div className="flex justify-between items-baseline text-[9px] font-medium">
                  <span className="text-slate-400">JNT ACCEL</span>
                  <span className={`font-mono font-semibold ${disableSpeed ? 'text-slate-500' : 'text-green-400'}`}>{accJ}%</span>
                </div>
                <input
                  type="range"
                  disabled={disableSpeed}
                  className={`slider-compact w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer text-green-500 ${
                    disableSpeed ? 'cursor-not-allowed' : ''
                  }`}
                  min="1"
                  max="100"
                  value={accJ}
                  onChange={(e) => setAccJ(Number(e.target.value))}
                  onPointerUp={handleSpeedUpdate}
                  onKeyUp={handleSpeedUpdate}
                />
              </div>
            </div>

            {/* Gripper Control Widget (钧舵 EPG50-060 / 50mm Parallel Gripper) — compact */}
            <div className={`bg-slate-950/80 border rounded-lg p-1.5 flex flex-col gap-1.5 flex-1 transition-all ${
              isGripperConnected ? 'border-slate-800' : 'border-slate-800/60 opacity-60'
            }`}>
              {/* Header: model + state badge + live stroke/force readout */}
              <div className="flex items-center justify-between gap-1 flex-wrap">
                <div className="flex items-center gap-1 min-w-0">
                  <span className="text-[9px] font-bold tracking-wide text-slate-300 shrink-0">GRIPPER</span>
                  {gripperSpecs?.model && (
                    <span className="text-[8px] px-1 rounded font-mono bg-slate-900 text-slate-400 border border-slate-800 truncate">
                      {gripperSpecs.model}
                    </span>
                  )}
                  <span className={`text-[8.5px] px-1 rounded font-mono uppercase font-semibold shrink-0 ${
                    isGripperConnected
                      ? 'bg-emerald-950 text-emerald-400 border border-emerald-800/60'
                      : 'bg-red-950/60 text-red-400 border border-red-800/60'
                  }`}>
                    {isGripperConnected ? (displayState.gripper?.state || 'IDLE') : 'DISCONNECTED'}
                  </span>
                </div>
                <div className="flex items-center gap-1.5 font-mono text-[9px] shrink-0">
                  <span className="text-slate-400">
                    STROKE: <strong className={isGripperConnected ? 'text-amber-400' : 'text-slate-500'}>
                      {(displayState.gripper?.position_mm ?? targetStroke).toFixed(1)}
                    </strong> mm
                  </span>
                  {displayState.gripper?.force_n !== undefined && displayState.gripper.force_n > 0 && isGripperConnected && (
                    <span className="text-blue-400 font-semibold">
                      {displayState.gripper.force_n.toFixed(1)} N
                    </span>
                  )}
                </div>
              </div>

              {/* Primary Stroke: Clamp / Slider / Open */}
              <div className="flex items-center gap-1.5">
                <button
                  onClick={handleGripperClamp}
                  disabled={disableGripper}
                  className={`px-1.5 py-1 text-[9px] font-semibold rounded border transition-colors select-none shrink-0 ${
                    disableGripper
                      ? 'bg-slate-900 border-slate-800 text-slate-600 cursor-not-allowed'
                      : 'bg-slate-800 hover:bg-slate-700 text-slate-200 border-slate-700'
                  }`}
                  title="Clamp to 0mm"
                >
                  CLAMP
                </button>

                <input
                  type="range"
                  min="0"
                  max={maxStrokeMm}
                  step="0.5"
                  value={targetStroke}
                  disabled={disableGripper}
                  onChange={(e) => setTargetStroke(Number(e.target.value))}
                  onPointerUp={() => handleGripperMove(targetStroke)}
                  onKeyUp={() => handleGripperMove(targetStroke)}
                  className={`slider-compact flex-1 min-w-0 h-1.5 bg-slate-800 rounded-lg appearance-none text-amber-500 ${
                    disableGripper ? 'opacity-30 cursor-not-allowed' : 'cursor-pointer'
                  }`}
                />

                <button
                  onClick={handleGripperOpen}
                  disabled={disableGripper}
                  className={`px-1.5 py-1 text-[9px] font-semibold rounded border transition-colors select-none shrink-0 ${
                    disableGripper
                      ? 'bg-slate-900 border-slate-800 text-slate-600 cursor-not-allowed'
                      : 'bg-slate-800 hover:bg-slate-700 text-slate-200 border-slate-700'
                  }`}
                  title={`Open to ${maxStrokeMm}mm`}
                >
                  OPEN
                </button>
              </div>

              {/* Gripper Speed & Grip Force Setting Sliders */}
              <div className="flex gap-2 pt-1 border-t border-slate-800/60">
                <div className="flex-1 flex flex-col gap-0.5 min-w-0">
                  <div className="flex justify-between items-baseline text-[8.5px] font-medium">
                    <span className="text-slate-400">GRP SPEED</span>
                    <span className={`font-mono font-semibold ${isGripperConnected ? 'text-amber-400' : 'text-slate-500'}`}>
                      {gripperSpeed}%
                    </span>
                  </div>
                  <input
                    type="range"
                    min="1"
                    max="100"
                    value={gripperSpeed}
                    disabled={disableGripper}
                    onChange={(e) => setGripperSpeed(Number(e.target.value))}
                    className={`slider-compact w-full h-1 bg-slate-800 rounded-lg appearance-none text-amber-500 ${
                      disableGripper ? 'opacity-30 cursor-not-allowed' : 'cursor-pointer'
                    }`}
                  />
                </div>
                <div className="flex-1 flex flex-col gap-0.5 min-w-0">
                  <div className="flex justify-between items-baseline text-[8.5px] font-medium">
                    <span className="text-slate-400">GRP FORCE</span>
                    <span className={`font-mono font-semibold ${isGripperConnected ? 'text-blue-400' : 'text-slate-500'}`}>
                      {gripperForce}%
                      <span className="text-[7.5px] text-slate-500 font-normal"> ({(gripperForce * maxForceN / 100).toFixed(1)}N)</span>
                    </span>
                  </div>
                  <input
                    type="range"
                    min="1"
                    max="100"
                    value={gripperForce}
                    disabled={disableGripper}
                    onChange={(e) => setGripperForce(Number(e.target.value))}
                    className={`slider-compact w-full h-1 bg-slate-800 rounded-lg appearance-none text-blue-500 ${
                      disableGripper ? 'opacity-30 cursor-not-allowed' : 'cursor-pointer'
                    }`}
                  />
                </div>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default JogControlPanel;

import React, { useState, useEffect, useRef } from 'react';
import { ListTodo } from 'lucide-react';
import FloatingCameraZone from '../components/FloatingCameraZone';
import RobotZone from '../components/RobotZone';
import ConsoleLogZone from '../components/ConsoleLogZone';
import CalibrationOp from '../components/operations/CalibrationOp';
import InteractiveOp from '../components/operations/InteractiveOp';

import { WS_BASE } from '../config';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
  connected?: boolean;
  tcp_speed_actual?: number[];
  tcp_speed_mm_s?: number;
  qd_actual?: number[];
  load?: number;
  error_status?: number;
  tool_vector_actual?: number[];
  hand_type?: number[];
  tool_index?: number;
  run_queued_cmd?: number;
  velocity_ratio?: number;
  xyz_velocity_ratio?: number;
  r_velocity_ratio?: number;
  digital_outputs?: number[];
  digital_output_bits?: number;
  gripper?: {
    connected?: boolean;
    position_mm?: number;
    state?: string;
    force_n?: number;
    specs?: any;
  };
}

interface WorkspaceViewProps {
  activeTab: string;
  isCameraVisible?: boolean;
  setIsCameraVisible?: (visible: boolean) => void;
}

const WorkspaceView: React.FC<WorkspaceViewProps> = ({ 
  activeTab, 
  isCameraVisible = false, 
  setIsCameraVisible 
}) => {
  const [robotState, setRobotState] = useState<RobotState>({ pose: [0,0,0,0,0,0], joint: [0,0,0,0,0,0] });
  const [simJoints, setSimJoints] = useState<number[] | null>(null);
  const [activeTemplate, setActiveTemplate] = useState<string | null>(null);
  const [activePathState, setActivePathState] = useState<'raw' | 'auto' | 'poi' | 'auto_poi'>('raw');
  const [meshVersion, setMeshVersion] = useState<number>(Date.now());
  const [pathsVersion, setPathsVersion] = useState<number>(Date.now());
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket(`${WS_BASE}/api/robot/ws`);
      ws.onmessage = (e) => {
        try {
          const msg = JSON.parse(e.data);
          if (msg.type === 'robot_state' && msg.data) {
            setRobotState((prev) => ({
              ...prev,
              ...msg.data,
              pose: msg.data.pose ?? prev.pose ?? [0, 0, 0, 0, 0, 0],
              joint: msg.data.joint ?? prev.joint ?? [0, 0, 0, 0, 0, 0],
            }));
          }
        } catch {}
      };
      ws.onclose = () => setTimeout(connect, 2000);
      wsRef.current = ws;
    };
    connect();

    return () => wsRef.current?.close();
  }, []);

  const effectiveRobotState: RobotState = (activeTab === 'interactive' && simJoints)
    ? { ...robotState, joint: simJoints }
    : robotState;

  const renderOperationZone = () => {
    switch (activeTab) {
      case 'calib':
        return <CalibrationOp />;
      case 'interactive':
        return (
          <InteractiveOp 
            externalActiveTemplate={activeTemplate}
            onTemplateChange={(tpl) => setActiveTemplate(tpl)}
            onMeshUpdated={() => setMeshVersion(Date.now())}
            onPathsUpdated={() => setPathsVersion(Date.now())}
            onPathStateChange={(st) => setActivePathState(st)}
            onSimulationJointsChange={(joints) => setSimJoints(joints)}
          />
        );
      case 'task':
        return (
          <div className="p-8 text-slate-400 bg-slate-900/50 rounded-xl border border-slate-800 w-full h-full flex flex-col items-center justify-center gap-3">
            <ListTodo size={36} className="text-slate-500" />
            <span className="text-sm font-medium">Task Management & Execution (Coming Soon)</span>
          </div>
        );
      default:
        return null;
    }
  };

  return (
    <div className="w-full h-full p-6 bg-slate-950 text-slate-200 overflow-hidden flex gap-6 relative">
      {/* Floating Camera Window */}
      {isCameraVisible && (
        <FloatingCameraZone onClose={() => setIsCameraVisible && setIsCameraVisible(false)} />
      )}
      
      {/* Left Column (60% width) - Operations & Logs */}
      <div className="flex-[3] flex flex-col gap-6 min-w-0 h-full">
        
        {/* Top Left: Operation Zone (Dynamic based on Tab) */}
        <div className="flex-1 min-h-0 flex flex-col">
          {renderOperationZone()}
        </div>

        {/* Bottom Left: Console Log Zone */}
        <div className="h-[420px] shrink-0 min-h-0">
          <ConsoleLogZone />
        </div>
      </div>

      {/* Right Column (40% width) - Robot Zone Full Height with Reconstructed 3D Mesh & 3D TCP Paths */}
      <div className="flex-[2] flex flex-col gap-6 min-w-0 h-full">
        {/* Full Right: Robot Zone (Only show workpiece mesh & paths when in Interactive tab) */}
        <div className="flex-1 min-h-0">
          <RobotZone 
            robotState={effectiveRobotState} 
            activeTemplate={activeTab === 'interactive' ? activeTemplate : null}
            meshVersion={meshVersion}
            pathsVersion={pathsVersion}
            pathState={activePathState}
          />
        </div>
      </div>
      
    </div>
  );
};

export default WorkspaceView;

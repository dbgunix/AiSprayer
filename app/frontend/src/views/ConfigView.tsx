import React, { useState, useEffect, useMemo } from 'react';
import {
  Save,
  Server,
  Camera as CameraIcon,
  Cpu,
  Layers,
  Scan,
  RotateCcw,
  Search,
  CheckCircle2,
  AlertCircle,
  Sliders,
  X,
  RefreshCw,
  Sparkles,
  Info
} from 'lucide-react';
import { API_BASE } from '../config';

interface ConfigOption {
  value: any;
  label: string;
}

interface ConfigItemMeta {
  key: string;
  category: 'robot' | 'calib' | 'spraying' | 'interactive';
  label: string;
  type: 'string' | 'number' | 'boolean' | 'select' | 'vector3' | 'tags';
  value: any;
  yaml_default: any;
  is_overridden: boolean;
  description?: string;
  min?: number;
  max?: number;
  step?: number;
  options?: ConfigOption[];
  legacy_key?: string;
}

interface CategoryMeta {
  id: 'robot' | 'calib' | 'spraying' | 'interactive';
  title: string;
  subtitle: string;
  icon: React.ComponentType<{ size?: number; className?: string }>;
  color: string;
  borderColor: string;
  bgGlow: string;
}

const CATEGORIES: CategoryMeta[] = [
  {
    id: 'robot',
    title: 'Robot Hardware & Tooling',
    subtitle: 'Controller IP, ports, speed scaling, TCP frame & kinematic URDF.',
    icon: Cpu,
    color: 'text-indigo-400',
    borderColor: 'border-indigo-500/20 hover:border-indigo-500/40',
    bgGlow: 'from-indigo-950/15 to-transparent',
  },
  {
    id: 'calib',
    title: 'Calibration Target & Mount',
    subtitle: 'Hand-eye mount, checkerboard rows/cols, square size & tolerance.',
    icon: CameraIcon,
    color: 'text-emerald-400',
    borderColor: 'border-emerald-500/20 hover:border-emerald-500/40',
    bgGlow: 'from-emerald-950/15 to-transparent',
  },
  {
    id: 'spraying',
    title: 'Spraying & Process Parameters',
    subtitle: 'Standoff distance, fan width, overlap, velocity & POI tolerance.',
    icon: Layers,
    color: 'text-amber-400',
    borderColor: 'border-amber-500/20 hover:border-amber-500/40',
    bgGlow: 'from-amber-950/15 to-transparent',
  },
  {
    id: 'interactive',
    title: 'Vision & Interactive SAM',
    subtitle: 'Garment pre-detection, MobileSAM refinement & model backends.',
    icon: Scan,
    color: 'text-purple-400',
    borderColor: 'border-purple-500/20 hover:border-purple-500/40',
    bgGlow: 'from-purple-950/15 to-transparent',
  },
];

// Wide items that need to span 2 columns inside a card
const WIDE_KEYS = new Set([
  'robot.robot_urdf',
  'spraying.poi_ref_rpy_deg',
  'spraying.poi_tolerance_rpy_deg',
  'interactive.detector.classes',
]);

const ConfigView: React.FC = () => {
  const [metadata, setMetadata] = useState<ConfigItemMeta[]>([]);
  const [formValues, setFormValues] = useState<Record<string, any>>({});
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [searchQuery, setSearchQuery] = useState('');
  const [activeCategory, setActiveCategory] = useState<string>('all');
  const [notification, setNotification] = useState<{ type: 'success' | 'error'; message: string } | null>(null);
  const [resetConfirmModal, setResetConfirmModal] = useState<{
    isOpen: boolean;
    type: 'all' | 'category' | 'key';
    target?: string;
    targetName?: string;
  }>({ isOpen: false, type: 'all' });

  // Load config metadata from API
  const fetchConfig = async () => {
    try {
      setLoading(true);
      const res = await fetch(`${API_BASE}/api/system/config`);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = await res.json();
      const metaList: ConfigItemMeta[] = data.metadata || [];
      setMetadata(metaList);

      const initialValues: Record<string, any> = {};
      metaList.forEach((item) => {
        initialValues[item.key] = item.value;
      });
      setFormValues(initialValues);
    } catch (err: any) {
      console.error('Failed to load system config:', err);
      showNotification('error', `Failed to load system configuration: ${err.message || err}`);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchConfig();
  }, []);

  const showNotification = (type: 'success' | 'error', message: string) => {
    setNotification({ type, message });
    setTimeout(() => {
      setNotification((prev) => (prev?.message === message ? null : prev));
    }, 4000);
  };

  // Track modified fields
  const modifiedKeys = useMemo(() => {
    const set = new Set<string>();
    metadata.forEach((item) => {
      const current = formValues[item.key];
      const original = item.value;
      if (JSON.stringify(current) !== JSON.stringify(original)) {
        set.add(item.key);
      }
    });
    return set;
  }, [formValues, metadata]);

  // Overall overridden count in DB
  const overriddenCount = useMemo(() => {
    return metadata.filter((m) => m.is_overridden).length;
  }, [metadata]);

  // Filter items by category and search term
  const filteredMetadata = useMemo(() => {
    return metadata.filter((item) => {
      if (activeCategory !== 'all' && item.category !== activeCategory) {
        return false;
      }
      if (!searchQuery.trim()) return true;
      const query = searchQuery.toLowerCase();
      return (
        item.label.toLowerCase().includes(query) ||
        item.key.toLowerCase().includes(query) ||
        (item.description && item.description.toLowerCase().includes(query))
      );
    });
  }, [metadata, activeCategory, searchQuery]);

  const handleFieldChange = (key: string, value: any) => {
    setFormValues((prev) => ({ ...prev, [key]: value }));
  };

  const handleSaveAll = async () => {
    setSaving(true);
    try {
      const payload: Record<string, any> = {};
      metadata.forEach((item) => {
        payload[item.key] = formValues[item.key];
      });

      const res = await fetch(`${API_BASE}/api/system/config`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ settings: payload }),
      });

      const data = await res.json();
      if (!res.ok) {
        throw new Error(data.detail || 'Failed to save configuration.');
      }

      if (data.metadata) {
        setMetadata(data.metadata);
        const updatedValues: Record<string, any> = {};
        data.metadata.forEach((m: ConfigItemMeta) => {
          updatedValues[m.key] = m.value;
        });
        setFormValues(updatedValues);
      }

      showNotification('success', 'System configurations saved and synchronized successfully.');
    } catch (err: any) {
      console.error('Save error:', err);
      showNotification('error', err.message || 'Failed to save configuration.');
    } finally {
      setSaving(false);
    }
  };

  const executeReset = async (type: 'all' | 'category' | 'key', target?: string) => {
    try {
      const body: Record<string, any> = {};
      if (type === 'key') {
        body.key = target;
      } else if (type === 'category') {
        body.category = target;
      } else {
        body.all = true;
      }

      const res = await fetch(`${API_BASE}/api/system/config/reset`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });

      const data = await res.json();
      if (!res.ok) {
        throw new Error(data.detail || 'Failed to reset configuration.');
      }

      if (data.metadata) {
        setMetadata(data.metadata);
        const updatedValues: Record<string, any> = {};
        data.metadata.forEach((m: ConfigItemMeta) => {
          updatedValues[m.key] = m.value;
        });
        setFormValues(updatedValues);
      }

      let msg = 'Configurations reset to YAML baseline defaults.';
      if (type === 'key') msg = `Reset "${target}" to YAML baseline default.`;
      if (type === 'category') msg = `Reset all "${target}" settings to YAML baseline defaults.`;
      showNotification('success', msg);
    } catch (err: any) {
      console.error('Reset error:', err);
      showNotification('error', err.message || 'Failed to reset configuration.');
    } finally {
      setResetConfirmModal({ isOpen: false, type: 'all' });
    }
  };

  const formatDefaultVal = (val: any) => {
    if (val === null || val === undefined) return 'None';
    if (typeof val === 'boolean') return val ? 'true' : 'false';
    if (Array.isArray(val)) {
      if (val.length <= 3) return `[${val.join(',')}]`;
      return `[${val.slice(0, 2).join(',')},...]`;
    }
    const str = String(val);
    return str.length > 18 ? str.slice(0, 15) + '...' : str;
  };

  const renderInputWidget = (item: ConfigItemMeta) => {
    const val = formValues[item.key];

    switch (item.type) {
      case 'boolean': {
        const checked = Boolean(val);
        return (
          <div className="flex items-center gap-2 py-0.5">
            <button
              type="button"
              role="switch"
              aria-checked={checked}
              onClick={() => handleFieldChange(item.key, !checked)}
              className={`relative inline-flex h-5 w-9 flex-shrink-0 cursor-pointer rounded-full border border-transparent transition-colors duration-200 ease-in-out focus:outline-none ${
                checked ? 'bg-blue-600' : 'bg-slate-700'
              }`}
            >
              <span
                className={`pointer-events-none inline-block h-4 w-4 transform rounded-full bg-white shadow ring-0 transition duration-200 ease-in-out ${
                  checked ? 'translate-x-4' : 'translate-x-0.5'
                }`}
              />
            </button>
            <span className={`text-xs font-mono font-medium ${checked ? 'text-blue-400' : 'text-slate-500'}`}>
              {checked ? 'ENABLED' : 'DISABLED'}
            </span>
          </div>
        );
      }

      case 'select': {
        return (
          <select
            value={val ?? ''}
            onChange={(e) => {
              const selectedValue =
                typeof item.yaml_default === 'number' ? Number(e.target.value) : e.target.value;
              handleFieldChange(item.key, selectedValue);
            }}
            className="w-full bg-slate-900/90 border border-slate-700/80 rounded-md px-2.5 py-1 text-xs text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
          >
            {item.options?.map((opt) => (
              <option key={String(opt.value)} value={opt.value}>
                {opt.label}
              </option>
            ))}
          </select>
        );
      }

      case 'vector3': {
        const arr = Array.isArray(val) && val.length === 3 ? val : [0, 0, 0];
        const labels = ['Rx', 'Ry', 'Rz'];
        return (
          <div className="grid grid-cols-3 gap-1.5">
            {labels.map((axis, i) => (
              <div key={axis} className="relative">
                <span className="absolute left-2 top-1 text-[10px] font-mono text-slate-500 font-bold uppercase">
                  {axis}
                </span>
                <input
                  type="number"
                  step="0.5"
                  value={arr[i] ?? 0}
                  onChange={(e) => {
                    const newArr = [...arr];
                    newArr[i] = parseFloat(e.target.value) || 0;
                    handleFieldChange(item.key, newArr);
                  }}
                  className="w-full bg-slate-900/90 border border-slate-700/80 rounded-md pl-7 pr-1.5 py-1 text-xs text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
                />
              </div>
            ))}
          </div>
        );
      }

      case 'tags': {
        const tags: string[] = Array.isArray(val) ? val : [];
        const rawString = tags.join(', ');
        return (
          <div className="flex items-center gap-2">
            <input
              type="text"
              value={rawString}
              onChange={(e) => {
                const list = e.target.value
                  .split(',')
                  .map((s) => s.trim())
                  .filter(Boolean);
                handleFieldChange(item.key, list);
              }}
              placeholder="e.g. trousers, shirt"
              className="flex-1 bg-slate-900/90 border border-slate-700/80 rounded-md px-2.5 py-1 text-xs text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
            />
            {tags.length > 0 && (
              <span className="text-[11px] font-mono text-slate-400 bg-slate-800 px-1.5 py-0.5 rounded shrink-0">
                {tags.length} class{tags.length > 1 ? 'es' : ''}
              </span>
            )}
          </div>
        );
      }

      case 'number': {
        return (
          <input
            type="number"
            min={item.min}
            max={item.max}
            step={item.step || 'any'}
            value={val ?? ''}
            onChange={(e) => {
              const num = e.target.value === '' ? '' : Number(e.target.value);
              handleFieldChange(item.key, num);
            }}
            className="w-full bg-slate-900/90 border border-slate-700/80 rounded-md px-2.5 py-1 text-xs text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
          />
        );
      }

      case 'string':
      default: {
        return (
          <input
            type="text"
            value={val ?? ''}
            onChange={(e) => handleFieldChange(item.key, e.target.value)}
            className="w-full bg-slate-900/90 border border-slate-700/80 rounded-md px-2.5 py-1 text-xs text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
          />
        );
      }
    }
  };

  if (loading) {
    return (
      <div className="flex-1 flex items-center justify-center p-6 bg-slate-950">
        <div className="text-slate-400 flex flex-col items-center gap-3">
          <div className="p-3 bg-slate-900 rounded-xl border border-slate-800 shadow-xl">
            <Server className="animate-pulse text-blue-500" size={28} />
          </div>
          <div className="text-center">
            <h3 className="text-sm font-medium text-slate-200">Loading Configuration Engine</h3>
            <p className="text-[11px] text-slate-500 mt-0.5">Connecting to SQLite registry & YAML baseline...</p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="flex-1 flex flex-col bg-slate-950 p-4 md:p-6 overflow-y-auto custom-scrollbar">
      <div className="max-w-7xl mx-auto w-full space-y-3.5">
        {/* Compact Streamlined Header Toolbar */}
        <div className="flex flex-wrap items-center justify-between gap-3 pb-3 border-b border-slate-800/80">
          <div className="flex items-center gap-2.5">
            <div className="p-1.5 bg-blue-500/10 border border-blue-500/20 rounded-lg text-blue-400 shrink-0">
              <Sliders size={18} />
            </div>
            <div>
              <div className="flex items-center gap-2">
                <h1 className="text-lg font-bold text-slate-100">System Configuration</h1>
                <span className="text-[10px] font-mono px-2 py-0.5 rounded-full bg-slate-800 text-slate-400">
                  {metadata.length} Parameters
                </span>
                {overriddenCount > 0 && (
                  <span className="text-[10px] font-mono px-2 py-0.5 rounded-full bg-amber-500/15 border border-amber-500/30 text-amber-300 font-medium flex items-center gap-1">
                    <Sparkles size={10} />
                    {overriddenCount} Customized
                  </span>
                )}
              </div>
              <p className="text-slate-500 text-[11px] leading-tight">
                3-Tier Cascading Engine: SQLite DB Overrides → YAML Defaults → Hardware Hot-Sync.
              </p>
            </div>
          </div>

          <div className="flex items-center gap-2 flex-wrap">
            {/* Search Input */}
            <div className="relative w-44 sm:w-56">
              <Search size={13} className="absolute left-2.5 top-2 text-slate-500" />
              <input
                type="text"
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                placeholder="Quick filter..."
                className="w-full bg-slate-900 border border-slate-800 rounded-md pl-7 pr-6 py-1 text-xs text-slate-200 placeholder:text-slate-500 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-sans"
              />
              {searchQuery && (
                <button
                  onClick={() => setSearchQuery('')}
                  className="absolute right-2 top-1.5 text-slate-500 hover:text-slate-300"
                >
                  <X size={11} />
                </button>
              )}
            </div>

            <button
              onClick={() =>
                setResetConfirmModal({
                  isOpen: true,
                  type: 'all',
                  targetName: 'All System Settings',
                })
              }
              title="Reset all dynamic overrides back to YAML baseline defaults"
              className="px-2.5 py-1 rounded-md text-xs font-medium bg-slate-900 hover:bg-slate-800 border border-slate-700/80 text-slate-300 hover:text-white transition-all flex items-center gap-1.5 shadow-sm"
            >
              <RotateCcw size={12} className="text-slate-400" />
              Reset All
            </button>

            <button
              onClick={handleSaveAll}
              disabled={saving}
              className={`px-3 py-1 rounded-md text-xs font-semibold shadow transition-all flex items-center gap-1.5 ${
                modifiedKeys.size > 0
                  ? 'bg-blue-600 hover:bg-blue-500 text-white shadow-blue-900/30'
                  : 'bg-slate-800 text-slate-400 hover:bg-slate-700'
              } disabled:opacity-50 disabled:cursor-not-allowed`}
            >
              {saving ? (
                <>
                  <RefreshCw size={12} className="animate-spin" />
                  Saving...
                </>
              ) : (
                <>
                  <Save size={12} />
                  Save Changes {modifiedKeys.size > 0 ? `(${modifiedKeys.size})` : ''}
                </>
              )}
            </button>
          </div>
        </div>

        {/* Global Toast Notification */}
        {notification && (
          <div
            className={`flex items-center justify-between px-3 py-2 rounded-lg border text-xs font-medium animate-in fade-in slide-in-from-top-1 duration-150 ${
              notification.type === 'success'
                ? 'bg-emerald-950/40 border-emerald-500/30 text-emerald-300'
                : 'bg-rose-950/40 border-rose-500/30 text-rose-300'
            }`}
          >
            <div className="flex items-center gap-2">
              {notification.type === 'success' ? (
                <CheckCircle2 size={14} className="text-emerald-400 shrink-0" />
              ) : (
                <AlertCircle size={14} className="text-rose-400 shrink-0" />
              )}
              <span>{notification.message}</span>
            </div>
            <button
              onClick={() => setNotification(null)}
              className="text-slate-400 hover:text-white transition-colors"
            >
              <X size={12} />
            </button>
          </div>
        )}

        {/* Category Filter Pills */}
        <div className="flex items-center gap-1.5 overflow-x-auto pb-0.5">
          <button
            onClick={() => setActiveCategory('all')}
            className={`px-2.5 py-1 rounded-md text-xs font-medium transition-all ${
              activeCategory === 'all'
                ? 'bg-blue-600 text-white shadow-sm'
                : 'text-slate-400 hover:text-slate-200 bg-slate-900/60 hover:bg-slate-800'
            }`}
          >
            All ({metadata.length})
          </button>
          {CATEGORIES.map((cat) => {
            const count = metadata.filter((m) => m.category === cat.id).length;
            const customCount = metadata.filter((m) => m.category === cat.id && m.is_overridden).length;
            return (
              <button
                key={cat.id}
                onClick={() => setActiveCategory(cat.id)}
                className={`px-2.5 py-1 rounded-md text-xs font-medium transition-all whitespace-nowrap flex items-center gap-1.5 ${
                  activeCategory === cat.id
                    ? 'bg-slate-800 text-white border border-slate-700'
                    : 'text-slate-400 hover:text-slate-200 bg-slate-900/60 hover:bg-slate-800/80'
                }`}
              >
                <span>{cat.title.split(' ')[0]} ({count})</span>
                {customCount > 0 && (
                  <span className="w-1.5 h-1.5 rounded-full bg-amber-400" />
                )}
              </button>
            );
          })}
        </div>

        {/* Ultra-Compact 4 Cards Grid */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-3.5 items-start">
          {CATEGORIES.filter((cat) => activeCategory === 'all' || activeCategory === cat.id).map((cat) => {
            const items = filteredMetadata.filter((m) => m.category === cat.id);
            if (items.length === 0 && searchQuery) return null;

            const categoryCustomizedCount = items.filter((m) => m.is_overridden).length;
            const IconComponent = cat.icon;

            return (
              <div
                key={cat.id}
                className={`bg-slate-900/50 backdrop-blur-sm border rounded-xl p-3.5 md:p-4 shadow-sm flex flex-col transition-all bg-gradient-to-b ${cat.bgGlow} ${cat.borderColor}`}
              >
                {/* Card Header (Compact) */}
                <div className="flex items-center justify-between gap-2 pb-2.5 mb-3 border-b border-slate-800/80">
                  <div className="flex items-center gap-2 min-w-0">
                    <div className={`p-1.5 rounded-lg bg-slate-950 border border-slate-800 ${cat.color} shrink-0`}>
                      <IconComponent size={15} />
                    </div>
                    <div className="min-w-0">
                      <div className="flex items-center gap-1.5">
                        <h2 className="text-sm font-semibold text-slate-100 truncate">{cat.title}</h2>
                        {categoryCustomizedCount > 0 && (
                          <span className="text-[9px] font-mono px-1.5 py-0.2 rounded bg-amber-500/10 text-amber-300 border border-amber-500/20 shrink-0">
                            {categoryCustomizedCount} custom
                          </span>
                        )}
                      </div>
                      <p className="text-[11px] text-slate-500 truncate leading-tight">{cat.subtitle}</p>
                    </div>
                  </div>

                  <button
                    onClick={() =>
                      setResetConfirmModal({
                        isOpen: true,
                        type: 'category',
                        target: cat.id,
                        targetName: cat.title,
                      })
                    }
                    title={`Reset all ${cat.title} parameters to YAML baseline`}
                    className="text-[10px] text-slate-400 hover:text-amber-400 flex items-center gap-1 transition-colors px-2 py-0.5 rounded bg-slate-950/80 border border-slate-800 hover:border-slate-700 shrink-0"
                  >
                    <RotateCcw size={10} />
                    Reset
                  </button>
                </div>

                {/* 2-Column Field Grid inside Card */}
                <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
                  {items.map((item) => {
                    const isOverridden = item.is_overridden;
                    const isLocallyModified = modifiedKeys.has(item.key);
                    const isWide = WIDE_KEYS.has(item.key);

                    return (
                      <div
                        key={item.key}
                        className={`${isWide ? 'sm:col-span-2' : 'col-span-1'} p-2 rounded-lg border transition-all flex flex-col justify-between ${
                          isLocallyModified
                            ? 'bg-blue-950/20 border-blue-500/40'
                            : isOverridden
                            ? 'bg-amber-950/10 border-amber-500/25'
                            : 'bg-slate-950/50 border-slate-800/60 hover:border-slate-700/80'
                        }`}
                      >
                        {/* Header: Label + Tooltip + Badges + Per-field Reset */}
                        <div className="flex items-center justify-between gap-1.5 mb-1.5">
                          <div className="flex items-center gap-1 min-w-0">
                            <label className="text-xs font-medium text-slate-200 truncate">
                              {item.label}
                            </label>
                            {item.description && (
                              <span
                                title={item.description}
                                className="text-slate-500 hover:text-slate-300 cursor-help shrink-0"
                              >
                                <Info size={11} />
                              </span>
                            )}
                          </div>

                          <div className="flex items-center gap-1 shrink-0">
                            {isLocallyModified ? (
                              <span className="text-[9px] font-mono font-medium px-1 py-0.2 rounded bg-blue-500/20 text-blue-300 border border-blue-500/30">
                                Unsaved
                              </span>
                            ) : isOverridden ? (
                              <span className="text-[9px] font-mono font-medium px-1 py-0.2 rounded bg-amber-500/15 text-amber-300 border border-amber-500/30">
                                Custom
                              </span>
                            ) : null}

                            {isOverridden && (
                              <button
                                onClick={() => executeReset('key', item.key)}
                                title="Reset to YAML baseline default"
                                className="text-[10px] text-slate-500 hover:text-amber-300 flex items-center gap-0.5 transition-colors px-1 py-0.5 rounded hover:bg-slate-800"
                              >
                                <RotateCcw size={9} />
                              </button>
                            )}
                          </div>
                        </div>

                        {/* Input Component */}
                        <div className="my-0.5">{renderInputWidget(item)}</div>

                        {/* Ultra-compact Footer: Key + YAML Default */}
                        <div className="flex items-center justify-between text-[10px] font-mono text-slate-500 mt-1 pt-1 border-t border-slate-900/80 leading-none">
                          <span className="truncate max-w-[130px]" title={item.key}>
                            {item.key}
                          </span>
                          <span className="text-slate-400 truncate max-w-[120px]" title={`YAML default: ${JSON.stringify(item.yaml_default)}`}>
                            def: {formatDefaultVal(item.yaml_default)}
                          </span>
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {/* Compact Confirmation Modal for Reset */}
      {resetConfirmModal.isOpen && (
        <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/75 backdrop-blur-xs animate-in fade-in duration-100">
          <div className="bg-slate-900 border border-slate-800 rounded-xl max-w-sm w-full p-4 shadow-2xl space-y-3">
            <div className="flex items-center gap-2.5">
              <div className="p-2 bg-amber-500/10 border border-amber-500/20 rounded-lg text-amber-400">
                <AlertCircle size={18} />
              </div>
              <div>
                <h3 className="text-sm font-semibold text-slate-100">
                  Confirm Baseline Reset
                </h3>
                <p className="text-[11px] text-slate-400">
                  Revert dynamic overrides back to YAML configuration.
                </p>
              </div>
            </div>

            <p className="text-xs text-slate-300 leading-relaxed bg-slate-950/60 p-2.5 rounded-lg border border-slate-800/80">
              Reset{' '}
              <strong className="text-amber-300">
                {resetConfirmModal.targetName || 'All System Settings'}
              </strong>{' '}
              to baseline YAML values? Dynamic SQLite overrides will be removed.
            </p>

            <div className="flex items-center justify-end gap-2 pt-1">
              <button
                onClick={() => setResetConfirmModal({ isOpen: false, type: 'all' })}
                className="px-3 py-1.5 rounded-md text-xs font-medium text-slate-400 hover:text-white bg-slate-800 hover:bg-slate-700 transition-colors"
              >
                Cancel
              </button>
              <button
                onClick={() =>
                  executeReset(resetConfirmModal.type, resetConfirmModal.target)
                }
                className="px-3 py-1.5 rounded-md text-xs font-semibold text-white bg-amber-600 hover:bg-amber-500 transition-colors shadow flex items-center gap-1"
              >
                <RotateCcw size={11} />
                Confirm Reset
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ConfigView;

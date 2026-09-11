from typing import Any, Dict, List, Optional
import json
import logging
import os
import sys
from contextlib import contextmanager

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from db.models import SysSettings
from db.database import SessionLocal

logger = logging.getLogger(__name__)


class SettingService:
    """
    SQLite 系统配置持久化管理服务。
    负责读写 sys_settings 表，支持 JSON 序列化、按命名空间分类与覆盖重置。
    """
    _db_initialized = False

    @classmethod
    def _ensure_initialized(cls):
        if cls._db_initialized:
            return
        try:
            from db.database import engine, Base
            from sqlalchemy import text
            Base.metadata.create_all(bind=engine)
            with engine.begin() as conn:
                res = conn.execute(text("PRAGMA table_info(sys_settings)")).fetchall()
                col_names = [r[1] for r in res] if res else []
                if col_names and "category" not in col_names:
                    conn.execute(text("ALTER TABLE sys_settings ADD COLUMN category VARCHAR(32) DEFAULT 'common'"))
            cls._db_initialized = True
        except Exception as e:
            logger.warning(f"[SettingService] Failed to auto-migrate sys_settings: {e}")

    @contextmanager
    def _session_scope(self):
        """提供事务范围与自动资源释放的上下文管理器。"""
        self._ensure_initialized()
        session = SessionLocal()
        try:
            yield session
            session.commit()
        except Exception as e:
            session.rollback()
            logger.error(f"[SettingService] Database session error: {e}")
            raise
        finally:
            session.close()

    def get_value(self, key: str, default: Any = None) -> Any:
        """读取指定 key 的动态配置，不存在时返回 default。"""
        with self._session_scope() as session:
            setting = session.query(SysSettings).filter(SysSettings.key == key).first()
            if not setting:
                return default
            try:
                return json.loads(setting.value)
            except (json.JSONDecodeError, TypeError):
                return setting.value

    def set_value(self, key: str, value: Any, category: Optional[str] = None, description: str = ""):
        """
        设置配置项值并持久化至 SQLite。
        若未显式指定 category，则自动从 key 的前缀推导 (例如 robot.ip -> category='robot')。
        """
        if category is None:
            category = key.split(".")[0] if "." in key else "common"

        # 统一采用 JSON 序列化保证复杂类型与基础类型（字符串、数字、布尔、数组）读写类型严格一致
        try:
            str_val = json.dumps(value)
        except Exception:
            str_val = str(value)

        with self._session_scope() as session:
            setting = session.query(SysSettings).filter(SysSettings.key == key).first()
            if setting:
                setting.value = str_val
                if category:
                    setting.category = category
                if description:
                    setting.description = description
            else:
                new_setting = SysSettings(key=key, value=str_val, category=category, description=description)
                session.add(new_setting)

    def delete_value(self, key: str) -> bool:
        """删除单个动态配置项（使其回退至 YAML 默认基线）。"""
        with self._session_scope() as session:
            setting = session.query(SysSettings).filter(SysSettings.key == key).first()
            if setting:
                session.delete(setting)
                return True
            return False

    def reset_category(self, category: str) -> int:
        """按分类清空动态配置项（例如清空所有 'spraying' 动态项）。"""
        with self._session_scope() as session:
            deleted_count = session.query(SysSettings).filter(SysSettings.category == category).delete()
            return deleted_count

    def reset_all(self) -> int:
        """清空所有动态配置覆盖，完全回归 YAML 基线。"""
        with self._session_scope() as session:
            deleted_count = session.query(SysSettings).delete()
            return deleted_count

    def get_all_settings(self) -> Dict[str, Any]:
        """获取所有动态配置的扁平字典 {key: parsed_value}。"""
        with self._session_scope() as session:
            settings = session.query(SysSettings).all()
            result = {}
            for s in settings:
                try:
                    result[s.key] = json.loads(s.value)
                except (json.JSONDecodeError, TypeError):
                    result[s.key] = s.value
            return result

    def get_all_records(self) -> List[Dict[str, Any]]:
        """获取所有动态配置元数据列表。"""
        with self._session_scope() as session:
            settings = session.query(SysSettings).all()
            records = []
            for s in settings:
                val = s.value
                try:
                    val = json.loads(s.value)
                except Exception:
                    pass
                records.append({
                    "key": s.key,
                    "value": val,
                    "category": s.category,
                    "description": s.description or "",
                    "updated_at": s.updated_at.isoformat() if s.updated_at else None
                })
            return records

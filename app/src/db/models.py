from sqlalchemy import Column, Integer, String, Float, Text, DateTime
from sqlalchemy.sql import func
from .database import Base

class SysSettings(Base):
    __tablename__ = "sys_settings"

    key = Column(String(64), primary_key=True, index=True)
    value = Column(Text, nullable=False)
    category = Column(String(32), index=True, default="common")
    description = Column(String(255), default="")
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), default=func.now())

class CalibRecord(Base):
    __tablename__ = "calib_records"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    timestamp = Column(DateTime(timezone=True), default=func.now())
    matrix_data = Column(Text)  # Serialized 4x4 matrix JSON
    reproj_error = Column(Float)
    samples_count = Column(Integer)

class PathTemplate(Base):
    __tablename__ = "path_templates"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    name = Column(String(100))
    type = Column(String(50))
    path_data = Column(Text)  # Serialized JSON array of points
    created_at = Column(DateTime(timezone=True), default=func.now())

#!/usr/bin/env python3
"""
STM32 Industrial Motion Controller - 统一构建脚本
支持多种构建模式和目标配置
"""

import os
import sys
import argparse
import subprocess
import json
from pathlib import Path

class BuildSystem:
    def __init__(self):
        self.project_root = Path(__file__).parent.parent.parent
        self.build_dir = self.project_root / "build"
        self.config_dir = self.project_root / "config" / "build"
        
        # 支持的构建模式
        self.modes = {
            'debug': 'Debug build with optimization disabled',
            'release': 'Release build with optimizations',
            'test': 'Test build with unit test support'
        }
        
        # 支持的目标
        self.targets = {
            'industrial': 'Industrial motion control (default)',
            'gcode': 'G-code parser test',
            'advanced': 'Advanced motion control test',
            'basic': 'Basic stepper motor control',
            'simple': 'Simple test program',
            'ui': 'UI test with LCD/Touch',
            'fsmc_test': 'FSMC basic test',
            'led_test': 'LED basic test'
        }

    def load_config(self, mode):
        """加载构建配置"""
        config_file = self.config_dir / f"{mode}.json"
        if config_file.exists():
            with open(config_file, 'r') as f:
                return json.load(f)
        return {}

    def clean(self):
        """清理构建目录"""
        if self.build_dir.exists():
            import shutil
            shutil.rmtree(self.build_dir)
            print(f"✓ 已清理构建目录: {self.build_dir}")

    def build(self, mode, target, clean_first=False, verbose=False):
        """执行构建"""
        print(f"🔨 开始构建...")
        print(f"   模式: {mode}")
        print(f"   目标: {target}")
        
        if clean_first:
            self.clean()
        
        # 创建构建目录
        build_subdir = self.build_dir / mode
        build_subdir.mkdir(parents=True, exist_ok=True)
        
        # 构建命令
        cmd = [
            "make",
            f"MODE={target}",
            f"DEBUG={'1' if mode == 'debug' else '0'}",
            f"BUILD_DIR={build_subdir}"
        ]
        
        if verbose:
            cmd.append("V=1")
        
        # 执行构建
        try:
            os.chdir(self.project_root)
            result = subprocess.run(cmd, check=True, 
                                  capture_output=not verbose,
                                  text=True)
            print("✓ 构建成功!")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"✗ 构建失败: {e}")
            if not verbose and e.stdout:
                print("输出:", e.stdout)
            if not verbose and e.stderr:
                print("错误:", e.stderr)
            return False

    def flash(self, mode, target):
        """烧录固件"""
        print(f"🔥 烧录固件...")
        
        cmd = ["make", "flash", f"MODE={target}"]
        
        try:
            os.chdir(self.project_root)
            subprocess.run(cmd, check=True)
            print("✓ 烧录成功!")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"✗ 烧录失败: {e}")
            return False

    def list_modes(self):
        """列出可用的构建模式"""
        print("可用的构建模式:")
        for mode, desc in self.modes.items():
            print(f"  {mode:10} - {desc}")

    def list_targets(self):
        """列出可用的构建目标"""
        print("可用的构建目标:")
        for target, desc in self.targets.items():
            print(f"  {target:12} - {desc}")

def main():
    parser = argparse.ArgumentParser(
        description="STM32 Industrial Motion Controller 构建系统",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                           # 构建默认配置 (debug, industrial)
  %(prog)s -m release -t gcode       # 构建G代码解析器发布版
  %(prog)s -c -f -v                  # 清理构建并烧录，显示详细信息
  %(prog)s --list-modes              # 列出可用模式
  %(prog)s --list-targets            # 列出可用目标
        """
    )
    
    parser.add_argument('-m', '--mode', 
                       choices=['debug', 'release', 'test'],
                       default='debug',
                       help='构建模式 (默认: debug)')
    
    parser.add_argument('-t', '--target',
                       choices=['industrial', 'gcode', 'advanced', 'basic', 
                               'simple', 'ui', 'fsmc_test', 'led_test'],
                       default='industrial',
                       help='构建目标 (默认: industrial)')
    
    parser.add_argument('-c', '--clean',
                       action='store_true',
                       help='构建前清理')
    
    parser.add_argument('-f', '--flash',
                       action='store_true',
                       help='构建后烧录')
    
    parser.add_argument('-v', '--verbose',
                       action='store_true',
                       help='显示详细构建信息')
    
    parser.add_argument('--list-modes',
                       action='store_true',
                       help='列出可用的构建模式')
    
    parser.add_argument('--list-targets',
                       action='store_true',
                       help='列出可用的构建目标')
    
    args = parser.parse_args()
    
    build_sys = BuildSystem()
    
    if args.list_modes:
        build_sys.list_modes()
        return
    
    if args.list_targets:
        build_sys.list_targets()
        return
    
    # 执行构建
    if build_sys.build(args.mode, args.target, args.clean, args.verbose):
        if args.flash:
            build_sys.flash(args.mode, args.target)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
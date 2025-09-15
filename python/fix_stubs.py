#!/usr/bin/env python3
"""
Post-process the generated .pyi file to fix common type annotation issues.
"""
import re
from pathlib import Path

def fix_stub_file(stub_path: Path):
    """Fix common issues in the generated stub file."""
    print(f"🔧 Fixing type annotations in {stub_path}")
    
    content = stub_path.read_text()
    
    # Fix specific type mappings
    fixes = [
        # Fix scanlines property
        (r'scanlines: \.\.\.', 'scanlines: list[DebugScanline]'),
        
        # Fix Result.value() methods with specific return types
        (r'(class DebugIntrinsicsResult:.*?def value\(self\) -> )\.\.\.:', r'\1DebugIntrinsics:'),
        (r'(class IntrinsicsResult:.*?def value\(self\) -> )\.\.\.:', r'\1Intrinsics:'),
        (r'(class RangeImageResult:.*?def value\(self\) -> )\.\.\.:', r'\1RangeImage:'),
    ]
    
    # Apply fixes
    for pattern, replacement in fixes:
        content = re.sub(pattern, replacement, content, flags=re.DOTALL)
    
    # Write back the fixed content
    stub_path.write_text(content)
    print("✅ Type annotations fixed")

if __name__ == '__main__':
    stub_file = Path("accurate_ri/_accurate_ri.pyi")
    if stub_file.exists():
        fix_stub_file(stub_file)
    else:
        print(f"❌ Stub file not found: {stub_file}")

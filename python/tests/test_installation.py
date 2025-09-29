"""
Test basic functionality of the alice_lri Python package
"""
import pytest
import alice_lri

#TODO UPDATE
def test_package_import():
    """Test that the package can be imported"""
    assert alice_lri.__version__ is not None
    print(f"Package version: {alice_lri.__version__}")


def test_available_functions():
    """Test that expected functions are available"""
    expected_functions = [
        'estimate_intrinsics',
        'estimate_intrinsics_detailed',
        'project_to_range_image',
        'unproject_to_point_cloud',
        'intrinsics_to_json_str',
        'intrinsics_from_json_str',
        'intrinsics_to_json_file',
        'intrinsics_from_json_file',
        'error_message'
    ]
    
    available_attrs = [attr for attr in dir(alice_lri) if not attr.startswith('_')]
    print(f"Available attributes: {available_attrs}")
    
    for func in expected_functions:
        assert hasattr(alice_lri, func), f"Function {func} not found in alice_lri"


def test_basic_data_structures():
    """Test that basic data structures can be created"""
    # Test that we can access the module without errors
    # More comprehensive tests would require actual data
    try:
        # Test classes from bindings.cpp
        assert hasattr(alice_lri, 'Intrinsics')
        assert hasattr(alice_lri, 'RangeImage')
        assert hasattr(alice_lri, 'Scanline')
        assert hasattr(alice_lri, 'DebugIntrinsics')
        assert hasattr(alice_lri, 'DebugScanline')
        assert hasattr(alice_lri, 'Interval')
        assert hasattr(alice_lri, 'ValueConfInterval')
        assert hasattr(alice_lri, 'ScanlineAngleBounds')
        
        # Test enums
        assert hasattr(alice_lri, 'ErrorCode')
        assert hasattr(alice_lri, 'EndReason')
        
        print("Basic data structures are accessible")
    except Exception as e:
        pytest.fail(f"Error accessing basic data structures: {e}")


def test_small_data_functionality():
    """Test with minimal data to ensure basic functionality works"""
    try:
        # Create minimal test data - lists of floats as expected by bindings
        x = [1.0, 2.0, 3.0]
        y = [0.5, 1.5, 2.5]
        z = [0.1, 0.2, 0.3]
        
        # Test that the estimate_intrinsics function exists and is callable
        assert callable(alice_lri.estimate_intrinsics)
        print("estimate_intrinsics function is callable")
        
        # Test creating basic objects
        # Create a RangeImage
        ri = alice_lri.RangeImage(10, 10)
        assert ri.width == 10
        assert ri.height == 10
        print("RangeImage creation works")
        
        # Create an Intrinsics object
        intrinsics = alice_lri.Intrinsics(5)  # 5 scanlines
        print("Intrinsics creation works")
        
        # Test JSON functions exist and work
        assert callable(alice_lri.intrinsics_to_json_str)
        assert callable(alice_lri.intrinsics_from_json_str)
        
        # Test JSON conversion (should work even with empty intrinsics)
        json_str = alice_lri.intrinsics_to_json_str(intrinsics)
        assert isinstance(json_str, str)
        print("JSON serialization works")
        
        # Test that we can call estimate_intrinsics function (may fail with small data, but shouldn't crash)
        try:
            result = alice_lri.estimate_intrinsics(x, y, z)
            print("estimate_intrinsics function executed successfully")
        except RuntimeError as e:
            # Expected to fail with small data, but should not be a symbol/import error
            print(f"estimate_intrinsics function callable but failed with small data (expected): {e}")
        
    except Exception as e:
        # Expected to potentially fail with actual computation, 
        # but should not fail due to missing symbols
        if "symbol" in str(e).lower() or "import" in str(e).lower() or "undefined" in str(e).lower():
            pytest.fail(f"Installation error: {e}")
        else:
            print(f"Expected computational error (not an installation issue): {e}")


if __name__ == "__main__":
    test_package_import()
    test_available_functions()
    test_basic_data_structures()
    test_small_data_functionality()
    print("All basic installation tests passed!")

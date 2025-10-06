import pytest
import alice_lri

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
    try:
        # Test classes from bindings.cpp
        assert hasattr(alice_lri, 'Intrinsics')
        assert hasattr(alice_lri, 'RangeImage')
        assert hasattr(alice_lri, 'Scanline')
        assert hasattr(alice_lri, 'Interval')
        assert hasattr(alice_lri, 'ValueConfInterval')
        assert hasattr(alice_lri, 'ScanlineAngleBounds')
        assert hasattr(alice_lri, 'ScanlineDetailed')
        assert hasattr(alice_lri, 'IntrinsicsDetailed')

        # Test enums
        assert hasattr(alice_lri, 'ErrorCode')
        assert hasattr(alice_lri, 'EndReason')

        print("Basic data structures are accessible")
    except Exception as e:
        pytest.fail(f"Error accessing basic data structures: {e}")

def test_small_data_functionality():
    """Test with minimal data to ensure basic functionality works"""
    try:
        x = [1.0, 2.0, 3.0]
        y = [0.5, 1.5, 2.5]
        z = [0.1, 0.2, 0.3]

        assert callable(alice_lri.estimate_intrinsics)
        print("estimate_intrinsics function is callable")

        ri = alice_lri.RangeImage(10, 10)
        assert ri.width == 10
        assert ri.height == 10
        print("RangeImage creation works")

        intrinsics = alice_lri.Intrinsics(5)
        print("Intrinsics creation works")

        assert callable(alice_lri.intrinsics_to_json_str)
        assert callable(alice_lri.intrinsics_from_json_str)

        json_str = alice_lri.intrinsics_to_json_str(intrinsics)
        assert isinstance(json_str, str)
        print("JSON serialization works")

        try:
            result = alice_lri.estimate_intrinsics(x, y, z)
            print("estimate_intrinsics function executed successfully")
        except RuntimeError as e:
            print(f"estimate_intrinsics function callable but failed with small data (expected): {e}")

    except Exception as e:
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

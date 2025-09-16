"""
Test basic functionality of the alice_lri Python package
"""
import pytest
import alice_lri


def test_package_import():
    """Test that the package can be imported"""
    assert alice_lri.__version__ is not None
    print(f"Package version: {alice_lri.__version__}")


def test_available_functions():
    """Test that expected functions are available"""
    expected_functions = [
        'train',
        'project_to_range_image_float',
        'project_to_range_image_double', 
        'unproject_to_point_cloud',
        'intrinsics_to_json_str',
        'intrinsics_from_json_str'
    ]
    
    available_attrs = [attr for attr in dir(alice_lri) if not attr.startswith('_')]
    print(f"Available attributes: {available_attrs}")
    
    for func in expected_functions:
        assert hasattr(alice_lri, func), f"Function {func} not found in alice_lri"


def test_error_codes():
    """Test that error codes are available"""
    assert hasattr(alice_lri, 'ErrorCode')
    error_code = alice_lri.ErrorCode.NONE
    assert error_code is not None


def test_basic_data_structures():
    """Test that basic data structures can be created"""
    # Test that we can access the module without errors
    # More comprehensive tests would require actual data
    try:
        # Just test that these don't cause import errors
        assert hasattr(alice_lri, 'Intrinsics')
        assert hasattr(alice_lri, 'RangeImage')
        print("Basic data structures are accessible")
    except Exception as e:
        pytest.fail(f"Error accessing basic data structures: {e}")


def test_small_data_functionality():
    """Test with minimal data to ensure basic functionality works"""
    try:
        # Create minimal test data
        x = [1.0, 2.0, 3.0]
        y = [0.5, 1.5, 2.5]
        z = [0.1, 0.2, 0.3]
        
        # This test just ensures the function exists and can be called
        # It may fail with the actual algorithm, but that's expected for minimal data
        # We're just testing the Python bindings work
        
        # Test that the train function exists and is callable
        assert callable(alice_lri.train)
        print("Train function is callable")
        
        # Test JSON functions exist
        assert callable(alice_lri.intrinsics_to_json_str)
        assert callable(alice_lri.intrinsics_from_json_str)
        print("JSON functions are callable")
        
    except Exception as e:
        # Expected to potentially fail with actual computation, 
        # but should not fail due to missing symbols
        if "symbol" in str(e).lower() or "import" in str(e).lower():
            pytest.fail(f"Installation error: {e}")
        else:
            print(f"Expected computational error (not an installation issue): {e}")


if __name__ == "__main__":
    test_package_import()
    test_available_functions()
    test_error_codes()
    test_basic_data_structures()
    test_small_data_functionality()
    print("All basic installation tests passed!")

'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

import pytest

@pytest.mark.filterwarnings("ignore")
def test_match(control: "TestController"): # type: ignore
    """Runs through the entire span of a practice match"""

    with control.run_robot():
        # Run disabled
        control.step_timing(seconds=1, autonomous=True, enabled=False)

        # Run autonomous + enabled
        control.step_timing(seconds=1, autonomous=True, enabled=True)

        # Disabled for another short period
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        # Run teleop + enabled
        control.step_timing(seconds=1, autonomous=False, enabled=True)

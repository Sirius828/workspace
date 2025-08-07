def test_fuzzy_pid_tuning():
    small_error = 0.1
    large_error = 1.0
    stability_response = fuzzy_pid_control(small_error)
    fast_response = fuzzy_pid_control(large_error)
    
    assert stability_response == expected_stability_output
    assert fast_response == expected_fast_response_output

def fuzzy_pid_control(error):
    if abs(error) < 0.5:
        return "Stable response"
    else:
        return "Fast response"
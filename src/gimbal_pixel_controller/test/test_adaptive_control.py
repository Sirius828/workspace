def test_adaptive_control():
    error_values = [0.1, 1.0, 5.0, 10.0]
    expected_outputs = [0.05, 0.5, 2.0, 5.0]  # 示例期望输出，根据误差大小调整

    for error, expected in zip(error_values, expected_outputs):
        output = adaptive_control_algorithm(error)
        assert abs(output - expected) < 0.1  # 允许的误差范围

def adaptive_control_algorithm(error):
    if abs(error) < 1.0:
        return error * 0.5  # 小误差时，控制输出较小
    else:
        return error * 0.5  # 大误差时，控制输出较大以快速响应

def test_debounce(robot, hal_data, control):
    def _on_step(tm):
        count = round(tm / 0.02)
        if count < 10:
            hal_data['joysticks'][0]['buttons'][2] = True
        elif count < 20:
            hal_data['joysticks'][0]['buttons'][2] = False
        else:
            hal_data['joysticks'][0]['buttons'][2] = True

        if count == 2 or count == 21:
            assert robot.debounce(2) is True
        else:
            assert robot.debounce(2) is False

        count += 1

        if count > 30:
            return False
        return True

    control.run_test(_on_step)

def perform_stage(stage, angle):

	if stage == 1:
		flight_time = compound_move_tello(tello, "down", 20, "forward", 170)
		return stage + 1, flight_time

	elif stage == 2:
		if angle[1] >= -0.05 and angle[1] <= 0.05:
			tello.send_rc_control(0, 0, 0, 0)
			stage = stage + 1

		elif angle[1] < -0.05:
			tello.send_rc_control(0, 0, 0, -10)

		elif angle[1] > 0.05:
			tello.send_rc_control(0, 0, 0, 10)

		return stage, 0.1

	elif stage == 3:
		flight_time = compound_move_tello(tello, "up", 50, "forward", 150)
		return stage + 1, flight_time

	elif stage == 4:
		tello.send_rc_control(0, 0, 0, 0)
		return stage + 1, 3

	elif stage == 5:
		flight_time = turn_tello(tello, 360)
		return stage + 1, flight_time

	elif stage == 6:
		tello.send_rc_control(0, 0, 0, 0)
		return stage + 1, 3

import fs2_neural
import fs2_baseline


def main():
	while (True):

		select = input("[N]eural? or [B]aseline? or [Q]uit?\n")
		if select == 'n' or select == 'N':
			print("Selected Neural FastSLAM")
			fs2_neural.run(10)
			break
		elif select == 'b' or select == 'B':
			print("Selected Baseline FastSLAM")
			fs2_baseline.run(10)
			break

		elif select == 'q' or select == 'Q':
			print("Quitting...")
			break
		else:
			print("Invalid input, please try again")


if __name__ == '__main__':
    main()
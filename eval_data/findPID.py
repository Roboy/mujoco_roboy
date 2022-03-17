import argparse



parser = argparse.ArgumentParser(
    description='Calculate PID values from Ultimate Gain Ku and Oscillation Periods.')
parser.add_argument('Ku', type=float,
                    help='Ultimate Gain, P value at which Ocscillations are self sustaining.')
parser.add_argument('N', type=float,
                    help='Number of oscillations (float) counted in the time window T')
parser.add_argument('T', type=float,
                    help='Time window [s] in which the oscillations have been counted')

args = parser.parse_args()

Ku = args.Ku
Tu = args.N/args.T
Kp = 0.6*Ku
Ki = 1.2*Ku/Tu
Kd = 0.075*Ku*Tu

print('Ku = %.4f' % Ku)
print('Tu = %.4f' % Tu)
print('GAINS:')
print('Kp = %.4f' % Kp)
print('Ki = %.4f' % Ki)
print('Kd = %.4f' % Kd)

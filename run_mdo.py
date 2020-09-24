def main():
    # imports
    import sys

    if len(sys.argv) >= 2:
        command = sys.argv[1]
        plane = sys.argv[2]
    else:
        command = ''
        plane = ''

    if command == 'report':
        aero_report(plane)
    else:
        print('plane not found')


def aero_report(plane_name):
    plane = __import__('src.airplanes.%s.plane' % plane_name, fromlist=['plane'])
    report = __import__('src.airplanes.%s.report' % plane_name, fromlist=['report'])
    report.report_sweep(plane.plane, name=plane_name)
    return


if __name__ == '__main__':
    main()

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
    elif command == 'design':
        aero_design(plane)
    else:
        print('plane not found')


def aero_report(plane_name):
    plane = __import__('src.airplanes.%s.plane' % plane_name, fromlist=['plane'])
    report = __import__('src.airplanes.report', fromlist=['report'])
    report.report_sweep(plane.plane, plane.requirements)
    return


def aero_design(plane_name):
    plane = __import__('src.airplanes.%s.plane' % plane_name, fromlist=['plane'])
    design = __import__('src.airplanes.design' % plane_name, fromlist=['design'])
    design.report_sweep(plane.plane, plane.requirements, name=plane_name)
    return


if __name__ == '__main__':
    main()

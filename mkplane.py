def main():
    # imports
    import sys

    if len(sys.argv) >= 2:
        command = sys.argv[1]
    else:
        command = ''

    n = 5

    if command == 'report':
        aero_report(n)
    else:
        print('plane not found')


def aero_report(n):
    from src.airplanes.example import report
    report
    print(n)
    return


if __name__ == '__main__':
    main()

import carla

def main():
    client = carla.Client('localhost', 2000)
    client.start_recorder(args.loggerPath, True)
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
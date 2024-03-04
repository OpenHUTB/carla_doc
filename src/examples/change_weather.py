import argparse
import carla


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0


class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0


class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--speed',
        metavar='FACTOR',
        default=1.0,
        type=float,
        help='rate at which the weather changes (default: 1.0)')
    argparser.add_argument(
        '-clouds',
        metavar='C',
        default=0,
        type=float,
        help='add clouds')
    argparser.add_argument(
        '-rain',
        metavar='R',
        default=0,
        type=float,
        help='add rain')
    argparser.add_argument(
        '-fog',
        metavar='F',
        default=0,
        type=float,
        help='add fog')
    argparser.add_argument(
        '-wind',
        metavar='W',
        default=0,
        type=float,
        help='add wind')
    argparser.add_argument(
        '-azimuth',
        metavar='az',
        default=5,
        type=float,
        help='change azimuth')
    argparser.add_argument(
        '-altitude',
        metavar='al',
        default=15,
        type=float,
        help='change altitude')
    argparser.add_argument(
        '-wetness',
        metavar='wet',
        default=2,
        type=float,
        help='add wetness')
    argparser.add_argument(
        '-puddles',
        metavar='pu',
        default=0,
        type=float,
        help='add puddles')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    weather = Weather(world.get_weather())
    weather._storm.clouds=args.clouds
    weather._storm.rain=args.rain
    weather._storm.fog=args.fog
    weather._storm.wind=args.wind
    weather._sun.azimuth=args.azimuth
    weather._sun.altitude=args.altitude
    weather._storm.wetness = args.wetness
    weather._storm.puddles = args.puddles

    weather.weather.cloudiness = weather._storm.clouds
    weather.weather.precipitation = weather._storm.rain
    weather.weather.fog_density = weather._storm.fog
    weather.weather.wind_intensity = weather._storm.wind
    weather.weather.sun_azimuth_angle = weather._sun.azimuth
    weather.weather.sun_altitude_angle = weather._sun.altitude
    weather.weather.wetness = weather._storm.wetness
    weather.weather.precipitation_deposits = weather._storm.puddles
    world.set_weather(weather.weather)

if __name__ == '__main__':
    main()

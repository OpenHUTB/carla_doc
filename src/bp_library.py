import carla

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

world = client.get_world()

blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
for blueprint in blueprints:
   print(blueprint.id)
   for attr in blueprint:
       print('  - {}'.format(attr))

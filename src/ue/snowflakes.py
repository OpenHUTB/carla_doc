import pygame
import random

# 初始化
pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()


# 雪花粒子类
class Snowflake:
    def __init__(self):
        self.x = random.randint(0, 800)
        self.y = random.randint(-50, 0)
        self.speed = random.uniform(1, 3)
        self.size = random.randint(2, 4)

    def update(self):
        self.y += self.speed
        if self.y > 600:
            self.y = random.randint(-50, 0)
            self.x = random.randint(0, 800)

    def draw(self):
        pygame.draw.circle(screen, (255, 255, 255), (self.x, self.y), self.size)


# 创建200片雪花
snowflakes = [Snowflake() for _ in range(200)]

# 主循环
running = True
while running:
    screen.fill((30, 30, 70))  # 深蓝色背景
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 更新和绘制雪花
    for flake in snowflakes:
        flake.update()
        flake.draw()

    pygame.display.flip()
    clock.tick(60)
pygame.quit()
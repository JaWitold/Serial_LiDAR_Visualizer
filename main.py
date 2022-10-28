import asyncio
import csv
import os
from datetime import datetime

import numpy as np
import pygame
import serial


class Scanner:
    SERIAL_PORT: str = 'COM6'
    SERIAL_BAUDRATE: int = 115200
    SERIAL_TIMEOUT: int = 0.1
    SAVE_PATH: str = "data"
    BUFFER_SIZE = 1000
    CANVAS_SIZE: int = 800
    LIDAR_RESOLUTION: int = 360

    distances_list = [0 for _ in range(LIDAR_RESOLUTION)]
    serial = None
    screen = None

    buffer_counter = 0

    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode([self.CANVAS_SIZE, self.CANVAS_SIZE])
        self.screen.fill(pygame.Color("gray23"))

        pygame.display.flip()

        self.prepare_save_dictionary()
        self.serial = serial.Serial(port=self.SERIAL_PORT, baudrate=self.SERIAL_BAUDRATE, timeout=self.SERIAL_TIMEOUT)

    def prepare_save_dictionary(self):
        for entity in os.listdir(self.SAVE_PATH):
            entity_path = os.path.join(self.SAVE_PATH, entity)

            if os.path.isfile(entity_path) or os.path.islink(entity_path):
                os.unlink(entity_path)

    async def receive(self):
        self.buffer_counter += 1
        tab = self.serial.readline().decode('ascii')
        tab = tab.rstrip().split(",")
        if len(tab) != 2 or tab[0] == '':
            return
        k, v = (int(tab[0]) + self.LIDAR_RESOLUTION) % self.LIDAR_RESOLUTION, int(tab[1])

        self.distances_list[k] = v
        await self.update()

        if self.buffer_counter >= self.BUFFER_SIZE:
            await self.save_to_csv()
            self.buffer_counter = 0
        # resp = check_keys()
        # arduino.write(str(resp).encode('ascii'))
        # print(resp)

    async def save_to_csv(self):
        now = datetime.now()
        file_name = f"{self.SAVE_PATH}/test_{now.strftime('%H_%M_%S')}.csv"
        with open(file_name, "w", newline='') as csv_file:
            write = csv.writer(csv_file)
            list_to_save = [[str(np.deg2rad(key)), str(v / 8192)] for key, v in enumerate(self.distances_list)]
            write.writerows(list_to_save)

    @staticmethod
    def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float):
        # map value from input range to corresponding value scaled and moved to output range
        return out_min + float(((value - in_min) / (in_max - in_min)) * (out_max - out_min))

    @staticmethod
    def polar_to_cartesian(r, eta):
        return r * np.cos(eta), r * np.sin(eta)

    async def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()

        self.screen.fill(pygame.Color("gray23"))
        offset = self.CANVAS_SIZE // 2
        center = (offset, offset)
        for a, r in enumerate(self.distances_list):
            r_mapped = self.map_range(r, 0, 8192, 0, offset)
            x, y = self.polar_to_cartesian(r_mapped, np.deg2rad(a))
            pygame.draw.aaline(self.screen,
                               pygame.Color("white"),
                               center,
                               (x + offset, y + offset),
                               )

        pygame.draw.circle(self.screen, pygame.Color("white"), center, 10)
        # Flip the display
        pygame.display.flip()

    async def main(self):
        while True:
            await self.receive()


if __name__ == "__main__":
    scanner = Scanner()
    asyncio.run(scanner.main())

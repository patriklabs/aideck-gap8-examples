import cflib.crtp
from cflib.cpx import CPXFunction, CPXTarget, CPXPacket
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.cpx import CPX
from cflib.cpx.transports import SocketTransport
import struct
import numpy as np
import cv2


class IMU:

    magic = 0xBE

    def decode(self, cpx_data):

        [magic, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, timestamp] = struct.unpack(
            "<BhhhhhhQ", cpx_data.data
        )

        if self.magic != magic:
            raise Exception("magic did not match for imu data")

        self.gyro_x = gyro_x / 500
        self.gyro_y = gyro_y / 500
        self.gyro_z = gyro_z / 500

        self.acc_x = acc_x / 1000
        self.acc_y = acc_y / 1000
        self.acc_z = acc_z / 1000
        self.timestamp = timestamp

    def print(self):
        print(
            f"{self.magic}, {self.gyro_x:.3f}, {self.gyro_y:.3f}, {self.gyro_z:.3f}, {self.acc_x:.3f}, {self.acc_y:.3f}, {self.acc_z:.3f}, {self.timestamp}"
        )


class ImageHeader:

    magic = 0xBC

    def decode(self, cpx_data):

        if len(cpx_data.data) != 19:
            raise Exception("image header data too long!")

        [magic, width, height, depth, format, size, timestamp] = struct.unpack(
            "<BHHBBIQ", cpx_data.data
        )

        if self.magic != magic:
            raise Exception("magic did not match for image header data")

        self.width = width
        self.height = height
        self.depth = depth
        self.format = format
        self.size = size
        self.timestamp = timestamp

    def print(self):
        print(
            f"{self.magic}, {self.width}, {self.height}, {self.depth}, {self.format}, {self.size}, {self.timestamp}"
        )


class VelocitySetpoint:

    def __init__(self, vel_x=0, vel_y=0, vel_z=0, yaw_rate=0) -> None:

        self.magic = 0xAA
        self.vel_x = vel_x * 1000  # short
        self.vel_y = vel_y * 1000  # short
        self.vel_z = vel_z * 1000  # short
        self.yaw_rate = yaw_rate * 500  # short

    def encode(self):
        return struct.pack(
            "<Bhhhh", self.magic, self.vel_x, self.vel_y, self.vel_z, self.yaw_rate
        )

    def decode(self, data):

        [self.magic, self.vel_x, self.vel_y, self.vel_z, self.yaw_rate] = struct.unpack(
            "<Bhhhh", data
        )

    def print(self):

        print(
            f"{self.magic}, {self.vel_x}, {self.vel_y}, {self.vel_z}, {self.yaw_rate}"
        )


class ImageData:

    magic = 0xAC

    def __init__(self, image_header: ImageHeader) -> None:
        self.image_header = image_header

        self.imgStream = bytearray()

    def decode(self, cpx_data):

        if cpx_data.data[0] != self.magic:
            raise Exception("magic did not match for image data")

        if len(cpx_data.data[1:]) + len(self.imgStream) <= self.image_header.size:
            self.imgStream.extend(cpx_data.data[1:])
        else:
            raise Exception("image data too long!")

    def complete(self):
        return len(self.imgStream) == self.image_header.size

    def get_image(self):
        if self.image_header.format == 0:
            bayer_img = np.frombuffer(self.imgStream, dtype=np.uint8)
            bayer_img.shape = (self.image_header.height, self.image_header.width)
            color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
            return color_img
        else:
            nparr = np.frombuffer(self.imgStream, np.uint8)
            decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)

            return decoded


def main():
    # Your code here
    cpx = CPX(SocketTransport("192.168.4.1", 5000))

    imu = IMU()
    image_header = ImageHeader()
    image_data = None

    count = 0

    while True:

        # Read the IMU data
        cpx_data = cpx.receivePacket(CPXFunction.APP)

        # Decode the data

        magic = cpx_data.data[0]

        if magic == IMU.magic:

            imu.decode(cpx_data)
            imu.print()

        elif magic == ImageHeader.magic:

            image_header.decode(cpx_data)
            image_header.print()

            image_data = ImageData(image_header)

        elif magic == ImageData.magic:

            if image_data is None:
                # raise Exception("image data received before image header")
                continue

            image_data.decode(cpx_data)

            if image_data.complete():
                image = image_data.get_image()
                cv2.imshow("Image", image)

                image_data = None

        cv2.waitKey(1)

        if count % 10 == 0:

            data = VelocitySetpoint(2, 3, 4, 5).encode()

            cpxPacket = CPXPacket(
                CPXFunction.CPX_F_CONTROL, CPXTarget.GAP8, CPXTarget.HOST, data
            )

            cpx.sendPacket(cpxPacket)

        count += 1


if __name__ == "__main__":
    main()

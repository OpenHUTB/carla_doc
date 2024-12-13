# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# 测试客户端链接服务端
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from . import SmokeTest


class TestClient(SmokeTest):
    def test_version(self):
        print("TestClient.test_version")
        # 判断客户端的版本和服务端的版本是否一致
        self.assertEqual(self.client.get_client_version(), self.client.get_server_version())

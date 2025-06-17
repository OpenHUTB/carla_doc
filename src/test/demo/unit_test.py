# 参考：https://blog.csdn.net/qq_40938678/article/details/105354002
import unittest

from add_test import add_test


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(add_test(), 12)  # 在这里增加断言


# 在Pycharm中右键点击"Run Python test in..."
# Launching unittests with arguments python -m unittest D:/work/workspace/doc/src/test/demo/unit_test.py
if __name__ == '__main__':
    unittest.main()

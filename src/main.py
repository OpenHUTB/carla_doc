# 需要 Python 3.9
# https://github.com/tassaron/get_code_from_markdown
import os

from get_code_from_markdown import *


def save_to_file(file_name, contents):
    fh = open(file_name, 'a')
    fh.write(contents)
    fh.write('\n\n')
    fh.close()


blocks = get_code_from_markdown_filename("../docs/tuto_G_pedestrian_navigation.md", language="py")

src_name = 'tuto_G_pedestrian_navigation.py'

if os.path.exists(src_name):
    os.remove(src_name)
for i in range(len(blocks)):
    save_to_file(src_name, blocks[i])
# run_code_from_markdown_blocks(blocks)


pass


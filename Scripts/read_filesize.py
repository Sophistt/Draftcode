import os
from os.path import join, getsize

FILEDIR = 'C:/Users/123/AppData/Local'


def getdirsize(dir):
    size = 0
    for root, dirs, files in os.walk(dir):
        size += sum([getsize(join(root, name)) for name in files])
    return size


if __name__ == '__main__':
    size_all = 0
    for dirs in os.listdir(FILEDIR):
        try:
            dir_size = getdirsize(join(FILEDIR, dirs))
            size_M = dir_size / 1024 / 1024
            if size_M > 100:
                print(dirs + ' size is %.3f M' % (size_M))
                size_all += size_M
        except OSError:
            pass
        continue

    print("All size is %.3f M" % (size_all))

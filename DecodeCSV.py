#coding=utf-8
import os
import shutil

rootdir = '/media/gi/Seagate Backup Plus Drive/北大西南山地红外相机数据-201808/'
destdir = '/home/gi/software/animal_analysis/'


def fetch_index(file_data):

    try:
        if "物种名称" in file_data[0].split(','):
            return  file_data[0].split(',').index('物种名称')
        elif "对象名称" in file_data[0].split(','):
            return  file_data[0].split(',').index('对象名称')
    except:
        if "物种名称" in file_data[0].split('\t'):
            return  file_data[0].split('\t').index('物种名称')
        elif "对象名称" in file_data[0].split('\t'):
            return  file_data[0].split('\t').index('对象名称')


#return decoded file data and its column number of '物种名称'
#in the form of a tuple (file, int)
def readcsv(csv):

    try:
        file_data = open(csv, encoding='GB2312').readlines()

        return file_data, fetch_index(file_data)
        
        # print("encoding='GB2312", csv)
    except:
        try:
            file_data = open(csv, encoding='utf-16').readlines()

            return file_data, fetch_index(file_data)

        except:

            try:
                file_data = open(csv, encoding='GB18030').readlines()

                return file_data, fetch_index(file_data)

            except:
                try:
                    file_data = open(csv, encoding='utf-8').readlines()

                    return file_data, fetch_index(file_data)

                except:

                    print("cannot decode : ", csv)






if __name__ == '__main__':
    file, idx = readcsv('/home/gishr/software/codes/readcsv/L-WL14-L15B.csv')

    print(file)

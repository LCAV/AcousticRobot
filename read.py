# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 11:43:36 2015

@author: kiki

Test reading files


"""
import csv


#%%
print('reader')
with open("test.txt") as csvfile:
    data = csv.reader(csvfile, delimiter='\t', skipinitialspace=True)
    for line_index, line in data:
        print(line_index, line)

#%%        
print('DictReader with names in code')
with open("test.txt") as csvfile:
    dictdata = csv.DictReader(csvfile, delimiter = '\t', fieldnames=['time','command'])
    for row in dictdata:
        print(row['time'])

#%%        
print('DictReader with names in file')
with open("testnames.txt") as csvfile:
    dictnames = csv.DictReader(csvfile, delimiter='\t')
    fieldnames = dictnames.fieldnames
    print(fieldnames)
    for row in dictnames:
            print(row[fieldnames[0]],row[fieldnames[1]])

#%%

"Dict synatxes"

a = dict(one=1, two=2, three=3)
b = {'one': 1, 'two': 2, 'three': 3}
c = dict(zip(['one', 'two', 'three'], [1, 2, 3]))
d = dict([('two', 2), ('one', 1), ('three', 3)])
e = dict({'three': 3, 'one': 1, 'two': 2})
a == b == c == d == e

dictlist = [];
counter = 0;
for written, number in a.items():
    temp = [written,number]
    dictlist.append(temp)
    print(written,number)
    print(dictlist[counter])
    counter = counter + 1
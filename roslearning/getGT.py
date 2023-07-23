with open('./gt.csv', 'r') as f:
    lines = f.readlines()[1:]
f.close()

with open('./gt.txt', 'w+') as f:
    for line in lines:
        items = line.split(',')
        f.write('%s %s %s %s %s %s %s %s\n'%(items[0][:10]+'.'+items[0][10:], items[1], items[2], items[3], items[4], items[5], items[6], items[7]))
f.close()
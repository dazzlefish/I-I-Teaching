def dfs(tree, count):
    for i in range(count):
        if(i in tree):
            entry = tree[i]
            print(i)
            if(entry != None):
                dfs(entry, count)

def bfs(tree, count):
    queue = []
    queue.insert(0, tree)

    while(not len(queue) == 0):
        nxt = queue.pop()
        for i in range(count):
            if(i in nxt):
                entry = nxt[i]
                print(i)
                if(entry != None):
                    queue.insert(0, entry)

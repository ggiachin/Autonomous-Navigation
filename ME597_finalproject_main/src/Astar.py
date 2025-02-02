from mapToGraph import Queue
import numpy as np
class AStar():
    def __init__(self,in_tree):
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name:np.Inf for name,node in in_tree.g.items()}
        self.h = {name:0 for name,node in in_tree.g.items()}
        
        for name,node in in_tree.g.items():
            start = tuple(map(int, name.split(',')))
            end = tuple(map(int, self.in_tree.end.split(',')))
            node_pos = tuple(map(int, name.split(',')))
            self.h[name] = np.sqrt((end[0]-node_pos[0])**2 + (end[1]-node_pos[1])**2)
        
        self.via = {name:0 for name,node in in_tree.g.items()}
        for __,node in in_tree.g.items():
            self.q.push(node)
     
    def __get_f_score(self,node):
      return self.dist[node.name] + self.h[node.name]
    
    def solve(self, sn, en):
      self.dist[sn.name] = 0
      while len(self.q) > 0:
          self.q.sort(key=self.__get_f_score)
          u = self.q.pop()
          if u.name == en.name:
            break
          for i in range(len(u.children)):
            c = u.children[i]
            w = u.weight[i]
            new_dist = self.dist[u.name] + w
            if new_dist < self.dist[c.name]:
              self.dist[c.name] = new_dist
              self.via[c.name] = u.name

    def reconstruct_path(self,sn,en):
        path = []
        dist = 0
        
        start_key = sn.name
        end_key = en.name
        dist = self.dist[end_key]
        u = end_key
        path = [u]
        while u != start_key:
            u = self.via[u]
            path.append(u)
        path.reverse()

        # Place code here
        return path,dist

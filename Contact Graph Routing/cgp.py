import numpy as np
from queue import PriorityQueue

'''
    class: Contact
    Creates contact nodes for every edge
'''
class Contact:
   
    def __init__(self, data, arr_time) -> None:
        [_, start_time, end_time, sender, receiver, owlt] = data
        self.start_time = float(start_time)
        self.end_time = float(end_time)
        self.sender = int(sender)
        self.receiver = int(receiver)
        self.owlt = float(owlt)
        self.arr_time = arr_time
        self.pred = None
        self.visited = False
        self.visited_n = []


def create_contact_list():
    '''
        Generates contact graph from the 
        given 190 contacts
    '''
    with open('ContactList.txt') as f:
        lines = f.readlines()

    q = PriorityQueue()
    contact_list = []
    for i, line in enumerate(lines):
        contact = Contact(line.replace("\n","").split(' '), np.inf)
        q.put((np.inf, i))
        contact_list.append(contact)

    return contact_list, q

class ContactGraph:
    def __init__(self) -> None:
        self.BDT = np.inf
        self.CFin = None
        self.Ccurr = Contact([0, 0, np.inf, 1, 1, 0], 0)
        self.G, self.q = create_contact_list()       
        self.TARGET = 12

    def CGR(self):
        '''
            Dijkstra's Algorithm for Contact Graph
        '''
        while True:
          (self.CFin, self.BDT) = self.CRP()
          self.Ccurr = self.CSP()
          if self.Ccurr == None:
              print("[INFO] Path found!!!")
              print()
              break
        
        while(self.CFin != None):
            print(self.CFin.receiver, end="->")
            self.CFin = self.CFin.pred
        print("root")
    
    def CRP(self):
        '''
            Contact Review Procedure - checks for appropriate neighbors
            and updates their arrival time if it is less than their current 
            arrival time.
        '''
        for i, C in enumerate(self.G):
            if C.sender != self.Ccurr.receiver or C.end_time < self.Ccurr.arr_time or C.visited == True or C.receiver in self.Ccurr.visited_n:
                continue
            arr_time = max(self.Ccurr.arr_time, C.start_time) + C.owlt
            if arr_time < C.arr_time:
                C.arr_time = arr_time
                C.pred = self.Ccurr
                C.visited = self.Ccurr.visited_n.append(C.receiver)
                self.q.put((arr_time, i))
                if C.receiver == self.TARGET and C.arr_time < self.BDT:
                    self.BDT = C.arr_time 
                    self.CFin = C
        self.Ccurr.visited = True
        return (self.CFin, self.BDT)
    
    # def CSP(self):
    #     self.Ccurr = None
    #     best_arr = np.inf
    #     for C in self.G:
    #         if C.arr_time > self.BDT or C.visited:
    #             continue
    #         if C.arr_time < best_arr:
    #             best_arr = C.arr_time
    #             self.Ccurr = C
        
    #     return self.Ccurr

    def CSP(self):
        '''
            Contact Selection Procedure - Selects the new current
            node for the exploration process.
        '''
        self.Ccurr = None
        curr_contact_val = self.q.get()[1]
        if self.G[curr_contact_val].visited or self.G[curr_contact_val].arr_time > self.BDT:
            while(not self.q.empty() and (self.G[curr_contact_val].visited == True)):
                curr_contact_val = self.q.get()[1]
                self.Ccurr = self.G[curr_contact_val]
                if self.Ccurr.arr_time > self.BDT:
                    return None
        else:
            return  self.G[curr_contact_val]
        return self.Ccurr     

if __name__ == '__main__':
    ContactGraph().CGR()
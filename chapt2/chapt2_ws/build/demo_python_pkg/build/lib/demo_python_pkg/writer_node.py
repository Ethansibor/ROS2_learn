from demo_python_pkg.person_node import PersonNode

class WriteNode(PersonNode):
    def __init__(self,name:str,age:int,book:str) -> None:
        super().__init__(name,age)
        print('WriteNode 的 __init方法被调用了')
        self.book = book

def main():
    node = WriteNode('张三',18,'论快速入狱')
    node.eat('鱼香肉丝')
        
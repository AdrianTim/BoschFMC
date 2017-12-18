class SaveEncoder:
    def __init__(self,fileName):
        self.fileName=fileName
        self.file=None
    def open(self):
        self.file=open(self.fileName,"w")
    def close(self):
        if not self.file==None:
            self.file.close()
            self.file=None
    def save(self,message):
        if not self.file==None:
            self.file.write(message+"\n")
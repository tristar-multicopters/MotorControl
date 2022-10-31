

### Requirements:
### 1- numpy
### 2- pandas
###3- matplotlib

import numpy as np
from tkinter.filedialog import askopenfilename
import pandas as pd
import matplotlib.pyplot as plt




def plot(xdata,ydata):
    plt.plot(xdata,ydata)
    plt.show

    

if __name__=="__main__":

    filename = askopenfilename()

    data = pd.read_csv(filename)

    df = pd.DataFrame(data)


    #df.plot()
    #plt.show()

    CurrentA = df['Current A']
    CurrentB = df['Current B']
    Idactual = df['Id actual']
    Idtarget = df['Id target']
    Iqactual = df['Iq actual']
    Iqtarget = df['Iq target']
    ELPOS = df['Electrical Pos']
    
    fig,axs = plt.subplots(4)
    fig.suptitle('High speed Log')
    axs[0].plot(CurrentA)
    axs[0].plot(CurrentB)
    axs[0].set_title('phase currents AB')
    axs[0].set(ylabel = 'current(A)')
    
    axs[1].plot(Idactual)
    axs[1].plot(Idtarget)
    axs[1].set_title('d-axis currents')
    axs[1].set(ylabel = 'current(A)')
    
    axs[2].plot(Iqactual)
    axs[2].plot(Iqtarget)
    axs[2].set_title('q-axis currents')
    axs[2].set(ylabel = 'current(A)')
    
    axs[3].plot(ELPOS)
    axs[3].set_title('Electrical position')
    axs[3].set(ylabel = 'position(rad)')
    plt.show()
    #CurrentA.plot(title='Current A')
    #plt.ylabel('Current (A)')
    #plt.show()

    
    #CurrentB.plot(title='Current B')
    #plt.ylabel('Current (A)')
    #plt.show()

    print (CurrentA[0])
    print(df.head())

#!/usr/bin/python3
# Let's Make the Gui To create the Barcode 

# using packages 
# pip install tk (tkinter)
# pip install python-barcode
# pip install pillow 


# Import it 
from tkinter import *
import barcode
import random
from barcode.writer import ImageWriter
from PIL.ImageTk import PhotoImage
import os


data = dict()


generatedBarcode  = []

# A fuction to genrate the different numbers for barcode 
def NumberGenerter():
    num1 = "0123456789"
    num2 = "0123456789"
    number  = num1 + num2
    length = 12
    result = "".join(random.sample(number,length))
    return result

def barcode_Generate():
    product = ProductNameEntry.get()
    price = PriceEntry.get()
    # set the barcode format 
    barcode_format = barcode.get_barcode_class('upc')
    # Barcode Number 
    barcodeNumber = NumberGenerter() #12 digit
    generated = barcode_format(barcodeNumber , writer=ImageWriter())
    home_dir = os.path.expanduser("~")
    generated.save(f'{home_dir}/Barcode_list/{product}')
    generatedBarcode.append(f"{home_dir}/Barcode_list/{product}.png")
    data[barcodeNumber] = [product, price , f"{home_dir}/Barcode_list/{product}.png"]
    

# lets make the gui 

Generator = Tk()

# Name the Gui Window 
Generator.title("Barcode Generator")

# size of Gui window 
Generator.geometry('600x600')


# Let's create the generate form and generate button

# product name 
ProductName = Label(Generator,text="Product Name",font=('bold',10))
ProductName.place(x=150 , y=50)

# Product Entry  
ProductNameEntry = Entry(Generator,width=20)
ProductNameEntry.place(x=270,y=50)




# Price
Price = Label(Generator,text="Price",font=('bold',10))
Price.place(x=150 , y=80)

# Price Entry  
PriceEntry = Entry(Generator,width=20)
PriceEntry.place(x=270,y=80)


# Button to generate the Barcode 
Button(Generator ,text='Generate', width=20, bg='brown',fg='white',font=('bold',10),command=barcode_Generate).place(x=200,y=130)

# Using label to show the barcode image 
image = Label(Generator)
image.place(x=50,y=200)

# show barcode product name
label = Label(Generator)
label.place(x=20 , y=550)


while True:
    # Let's get image and show 
    for img in generatedBarcode:
        img2=  PhotoImage(file=img)
        image['image'] = img2
        label['text'] = data
    Generator.update()
#!/usr/bin/python3

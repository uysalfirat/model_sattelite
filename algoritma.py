from time import sleep
i=1
altitude=[0]
kontrol=0
while True:
    altitude + [i] 
    if(altitude[len(altitude)-1] < 25 and kontrol==0):
        durum='beklemede'
        altitude[i-1]=5+altitude[len(altitude)-2]
        dongu=1
    elif(altitude[len(altitude)-1] >= 25)and(kontrol==0)and(altitude[len(altitude)-1]<500):
        durum= 'yükselme'
        altitude[i-1]=25+altitude[len(altitude)-2]
        dongu=2
    elif(altitude[len(altitude)-1]<=500)and(altitude[len(altitude)-1]>475)and(kontrol==0):
        altitude[i-1]=altitude[len(altitude)-1]-12.5
        durum='Model uydu iniş'
        dongu=3
        kontrol=1
    elif(altitude[len(altitude)-1]<=475)and(kontrol==1)and(altitude[len(altitude)-1]>400):
        durum='model uydu iniş'
        altitude[i-1]=altitude[len(altitude)-2]-25        
        dongu=4
    elif(altitude[len(altitude)-1]<=400)and(kontrol==1)and(altitude[len(altitude)-1]>=25):
        durum='ayrılma'
        altitude[i-1]=altitude[len(altitude)-2]-25 
        dongu=5
    elif(altitude[len(altitude)-1]==0)and(kontrol==1):
        durum='görev yükü başarıyla yere iniş yaptı'
        kontrol=2
    elif(kontrol==2):
        break
    
    print('yükseklik',altitude[len(altitude)-1],'metre',durum,'döngü:',dongu)
    sleep(1)
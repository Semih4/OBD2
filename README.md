# OBD2
Proje ismi SEDAS (Sürüş ve Engine Diagnostics Analiz Sistemi). OBD portu ile veri alıp bir ML modeli ile yorumlayıp sürücüye kolay erişim sağlamak amacı düşünülmüştür.
OBD portu ile araç motoru beynine  bağlanarak aracın beyninden (ECU) devir, manifold basıncı, yakıt sistemi, ateşleme verisi, araç şasi numarası (VİN),  arza kodları gibi veriler alınabilir bu verileri bir ML modeli kullanarak bağlantılı tahmin yürütme ve analiz oluşturma sistemi oluşturmayı hedefliyorum.
Araç içi yazılım ve sürüş analizi projeleri üzerinde yapılan çalışmalarda araca motor beyninden erişim sağlanmaktadır. ARAÇ beyninee OBD adı verilen erkek pinli söket üzerinden bağlantı kuruluyor ve Hız, Devir, Yakıt durumu, Manifold Basıncı gibi motor üzeri haberleşme ile alınabilen canlı veriler çekilebilir. ELM327 cihazı ile android cihazlara ble ile bağlantı oluşturup bu canlı verileri görebileceğimiz bazı uygulmalar mevcuttur. Detaylı araştırmalar sonucunda ELM327 cihazının yetersiz kaldığı ve bazen Darboğaz oluşumu gerçekleştiği gibi durumlar belirtilmektedir. Bu sorunun sebebi bazen ELM327 cihazının piyasa üzerinde genelde klon kartlarının bulunması bazen de ASCII karakter dönüşümü sebebi ile veri aktarım hızında düşüş yaşanması diye belirtiliyor. Bu sebeple kendi oluşturduğumuz sistem ile veri çekmeyi amaçlıyorum.
Projenin 1. aşaması olan veri çekme hedefi için bu alan üzerinde detaylı bilgi edinmek için  YökTez sayfasında yayınlanmış; Selçuk Üniversitesi Elektirik - Elektronik Yüksek Lisans adayı Kerim Dündar "Araç İçi Protokol Dönüştürücü Tasarımı ve Uygulaması " , Isparta SDÜ Doktora adayı Gül Fatma Türker'in Haberleşme Mühendisliği Anabilim Dalı'nda Doktora Tezi "MOBİL CİHAZLAR İLE SUNUCU TEMELLİ ARAÇLAR ARASI HABERLEŞME VE SÜRÜCÜ BİLGİLENDİRME SİSTEMİ" gibi kaynak araştırmaları sonucunda araç beyni üzerinden araç içi verileri çekmek için bir sistem oluşturmayı amaçlamaktayım. Oluşturmayı düşündüğüm sistemde öncelikle bir araç beyninin veri üretme tarzı, türü, ve haberleşme protokolü üzerinde deneyim kazanımı için ortalama bir ECU(Electronic Control Unit)'nun bağlantılı veri üretimi için bilgisayar ortamında bir ECU simülasyonu oluşturulacağım burda amacım motor veriyi hangi türde üretir, hangi verileri üretir ve ne sıklıkla üretir bunu görmek.
Bu simülasyonda edinmeyi hedeflediğim teorik bilgileri gerçek bir haberleşme sisteminde deneyimlemek için 2 adet mikro denetleyici ile veri çekme işlemi yapacağım bir tarafta motor beyni olarak belirlediğim sistem diğer mikrodenetleyici tarafında veri okuma sistemi oluşturmayı hedefleyerek kısaca iki mikroçip arasında SPI haberleşme protokolü ve araçlarda bulunan CAN-Bus protokolünü kullanarak veri transferi gerçekleştirmeyi hedefliyorum .




# ESP32 & MCP2515 Masaüstü OBD-II CAN Bus Simülasyonu

Bu proje, iki adet **ESP32** mikrodenetleyicisi ve iki adet **MCP2515** CAN modülü kullanılarak oluşturmayı hedeflediğim kapalı devre bir araç içi haberleşme (CAN Bus) sistemidir . 

Sistem, gerçek bir araca bağlanmadan önce ECU (Motor Kontrol Ünitesi) simülasyonu yapmak ve standart OBD-II PID sorgularını (Request/Response) test etmek amacıyla geliştirilmiştir. Bu altyapı, Edge-AI tabanlı sürüş analizi ve kestirimci bakım odaklı lisans tezi projesinin temel donanım ve haberleşme doğrulama aşamasını oluşturmaktadır.

##  Proje Mimarisi ve Çalışma Mantığı

Sistem iki ana düğümden (node) oluşmaktadır:
1. **Node 1 - ECU Simülatörü:** Gelen OBD-II standart `0x7DF` sorgularını dinler ve sahte motor verileri (RPM, Hız, Motor Sıcaklığı vb.) üreterek `0x7E8` ID'si ile hatta geri bırakır.
2. **Node 2 - OBD-II Okuyucu (Tarayıcı):** Belirli periyotlarla CAN hattına PID sorguları gönderir, dönen hex formatındaki paketleri yakalar ve anlamlı ondalık (decimal) telemetri verilerine dönüştürür.

##  Donanım Gereksinimleri

* 2x ESP32 Geliştirme Kartı
* 2x MCP2515 CAN Bus Modülü (Devre yanmasın daha doğrusu gelen güç sistemi yakmasın diye 120Ω sonlandırma dirençleri aktif)
* Jumper Kablolar (Erkek-Erkek / Dişi-Erkek)

### Pin Bağlantı Şeması (SPI Haberleşmesi)

| MCP2515 Pin | ESP32 Pin (Node 1 & 2) | Açıklama |
| :--- | :--- | :--- |
| **VCC** | 5V (veya 3.3V) | Güç beslemesi (Modül regülatörüne göre) |
| **GND** | GND | Toprak |
| **CS** | GPIO 5 | Chip Select |
| **SO (MISO)** | GPIO 19 | Master In Slave Out |
| **SI (MOSI)** | GPIO 23 | Master Out Slave In |
| **SCK** | GPIO 18 | Serial Clock |

*Not: İki MCP2515 modülünün `CAN-H` pinleri birbirine, `CAN-L` pinleri birbirine doğrudan bağlanmalıdır.*

##  Gelecek Geliştirmeler ve Hedefler (Roadmap)

- [ ] Temel Ping-Pong CAN haberleşmesinin doğrulanması.
- [ ] Okuyucu ESP32'nin gerçek bir aracın OBD2 portuna entegrasyonu.
- [ ] IMU (İvmeölçer) ve GPS modüllerinin sisteme dahil edilmesi.
- [ ] Edge-AI ile toplanan telemetri verileri üzerinden sürüş profili analizi.

## Geliştirici

**Muhammed Semih Erdoğan**  
Bilgisayar Mühendisliği Lisans Projesi Araştırma Deposu

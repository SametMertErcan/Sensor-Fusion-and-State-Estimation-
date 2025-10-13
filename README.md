# Sensor-Fusion-and-State-Estimation-
Sensor Fusion and State Estimation with Extended Kalman Filter (EKF)



ÖZET

Bu çalışmada, yaklaşık 10 dakikalık bir İHA uçuşu, kalkış–tırmanış–seyir–iniş gibi farklı fazları içerecek şekilde simüle edilecektir. Bu senaryodan üretilen yer gerçeği (ground truth) zaman serileri, sensör modellemesi ve filtre doğrulaması için referans olacaktır. Ardından AÖB , GNSS, manyetometre, barometre ve pitot tüpü sensörleri, gerçekçi gürültü (noise) ve kayıklık (bias) parametreleriyle; farklı örnekleme hızları nedeniyle asenkron ölçümler üretecek biçimde modellenecektir. İlgili hata/gürültü değerleri literatür ve piyasadaki düşük maliyetli sensör spesifikasyonlarından türetilecektir. 

Bu altyapı üzerinde Genişletilmiş Kalman Filtresi (Extended Kalman Filter, EKF) tasarlanacaktır. Tahmin, ilerletme (state prediction) ve güncelleme (measurement update) adımlarının denklemleri uygulanacak; durum geçiş fonksiyonu (state transition) ve buna karşılık gelen Jacobian’lar oluşturulacaktır. Başlangıç kovaryansı \((P_0)\) ile süreç gürültüsü kovaryansı \((Q)\) ve ölçüm gürültüsü kovaryansı \((R)\) tanımlanacak; yenilik (innovation) büyüklüğü izlenerek filtrenin kararlılığı ve güvenilirliği değerlendirilecektir. Hata kovaryansı güncellemesi hem Joseph formu hem de basitleştirilmiş form ile uygulanıp karşılaştırılacaktır. Çeşitli saha koşulları için koşullu güncellemeler tasarlanacak (ör. GNSS kesintisi/outage). Kayıklık (bias) kestirimi durum vektörüne entegre edilerek takip edilecektir. 

Son aşamada çıktılar görselleştirilip nicel olarak analiz edilecek; P ve Q matrislerinin seyri ile yenilik istatistikleri raporlanacaktır. Duruş (attitude) kestiriminde tamamlayıcı filtre (CF), Madgwick ve Kalman tabanlı yaklaşımlar aynı veri üzerinde karşılaştırılarak güçlü/zayıf yönleri tartışılacaktır. Çalışma, geliştirme olanakları kapsamında yapılabileceklerden bahsedilerek tamamlanacaktır. 


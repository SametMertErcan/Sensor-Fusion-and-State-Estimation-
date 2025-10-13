# Sensor-Fusion-and-State-Estimation-
Sensor Fusion and State Estimation with Extended Kalman Filter (EKF)



ÖZET

Bu çalışmada, yaklaşık 10 dakikalık bir İHA uçuşu, kalkış–tırmanış–seyir–iniş gibi farklı fazları içerecek şekilde simüle edilmiştir. Bu senaryodan üretilen yer gerçeği (ground truth) zaman serileri, sensör modellemesi ve filtre doğrulaması için referans olarak kullanılmıştır. Ardından AÖB, GNSS, manyetometre, barometre ve pitot tüpü sensörleri, gerçekçi gürültü (noise) ve kayıklık (bias) parametreleriyle; farklı örnekleme hızları nedeniyle asenkron ölçümler üretecek biçimde modellendirilmiştir. İlgili hata ve gürültü değerleri, literatür ve piyasadaki düşük maliyetli sensör spesifikasyonlarından türetilmiştir.

Bu altyapı üzerinde Genişletilmiş Kalman Filtresi (Extended Kalman Filter, EKF) tasarlanmıştır. Tahmin, ilerletme (state prediction) ve güncelleme (measurement update) adımlarının denklemleri uygulanmıştır; durum geçiş fonksiyonu (state transition) ve buna karşılık gelen Jacobian’lar oluşturulmuştur. Başlangıç kovaryansı (P₀) ile süreç gürültüsü kovaryansı (Q) ve ölçüm gürültüsü kovaryansı (R) tanımlanmıştır; yenilik (innovation) büyüklüğü izlenerek filtrenin kararlılığı ve güvenilirliği değerlendirilmiştir. Hata kovaryansı güncellemesi hem Joseph formu hem de basitleştirilmiş form ile uygulanmış ve karşılaştırılmıştır. Çeşitli saha koşulları için koşullu güncellemeler tasarlanmıştır (örneğin GNSS kesintisi/outage). Kayıklık (bias) kestirimi durum vektörüne entegre edilerek takip edilmiştir.

Son aşamada çıktılar görselleştirilmiş ve nicel olarak analiz edilmiştir; P ve Q matrislerinin seyri ile yenilik istatistikleri raporlanmıştır. Duruş (attitude) kestiriminde tamamlayıcı filtre (CF), Madgwick ve Kalman tabanlı yaklaşımlar aynı veri üzerinde karşılaştırılmış ve güçlü ile zayıf yönleri tartışılmıştır. Çalışma, geliştirme olanakları kapsamında yapılabileceklerden bahsedilerek tamamlanmıştır.

<img width="1026" height="769" alt="2-rota3dalt_cop (1)" src="https://github.com/user-attachments/assets/99237140-8b4d-4e34-9283-0731403d1587" /> 


![clideo_editor_c8b056c49360486cb553f914955f8e0a](https://github.com/user-attachments/assets/d4657182-8f55-4cb6-a15d-eef6b0c99471)


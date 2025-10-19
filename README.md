# Sensör-Füzyonu-ve-Durum-Kestirimi-Projesi
Genişletilmiş Kalman Filtresi (EKF) ile Sensör Füzyonu ve Durum Kestirimi

## ÖZET
Bu çalışmada, yaklaşık 10 dakikalık bir İHA uçuşu, kalkış–tırmanış–seyir–iniş gibi farklı fazları içerecek şekilde simüle edilmiştir. Bu senaryodan üretilen yer gerçeği (ground truth) zaman serileri, sensör modellemesi ve filtre doğrulaması için referans olarak kullanılmıştır. Ardından AÖB, GNSS, manyetometre, barometre ve pitot tüpü sensörleri, gerçekçi gürültü (noise) ve kayıklık (bias) parametreleriyle; farklı örnekleme hızları nedeniyle asenkron ölçümler üretecek biçimde modellendirilmiştir. İlgili hata ve gürültü değerleri, literatür ve piyasadaki düşük maliyetli sensör spesifikasyonlarından türetilmiştir.

Bu altyapı üzerinde Genişletilmiş Kalman Filtresi (Extended Kalman Filter, EKF) tasarlanmıştır, EKF NED koordinat çerçevesinde koşturulmuştur. Tahmin, ilerletme (state prediction) ve güncelleme (measurement update) adımlarının denklemleri uygulanmıştır; durum geçiş fonksiyonu (state transition) ve buna karşılık gelen Jacobian’lar oluşturulmuştur. Başlangıç kovaryansı (P₀) ile süreç gürültüsü kovaryansı (Q) ve ölçüm gürültüsü kovaryansı (R) tanımlanmıştır; yenilik (innovation) büyüklüğü izlenerek filtrenin kararlılığı ve güvenilirliği değerlendirilmiştir. Hata kovaryansı güncellemesi hem Joseph formu hem de basitleştirilmiş form ile uygulanmış ve karşılaştırılmıştır. Çeşitli saha koşulları için koşullu güncellemeler tasarlanmıştır (örneğin GNSS kesintisi/outage). Kayıklık (bias) kestirimi durum vektörüne entegre edilerek takip edilmiştir. Kovaryans matrisi (P) güncellemesi için farklı matematiksel formlar denenmiştir.

Son aşamada çıktılar görselleştirilmiş ve nicel olarak analiz edilmiştir; P ve Q matrislerinin seyri ile yenilik istatistikleri raporlanmıştır. Duruş (attitude) kestiriminde tamamlayıcı filtre (CF), Madgwick ve Kalman tabanlı yaklaşımlar aynı veri üzerinde karşılaştırılmış ve güçlü ile zayıf yönleri tartışılmıştır. Çalışma, geliştirme olanakları kapsamında yapılabileceklerden bahsedilerek tamamlanmıştır.

---

<p align="center">
  <img width="600" alt="3D flight trajectory" src="https://github.com/user-attachments/assets/99237140-8b4d-4e34-9283-0731403d1587" /><br>
  <em>Şekil 1.  Uçuş rotasının 3B görünümü ve fazların işaretlenmesi</em>
</p>

<p align="center">
  <img width="600" alt="sensor-fusion-block-diagram" src="https://github.com/user-attachments/assets/9dd52b5f-2969-499e-bf58-2230756dfbc4" /><br>
  <em>Şekil 2. Uçuş fazları ve süreleri</em>
</p>

---

<div align="center">
<table>
<tr>
<td align="center">
<img src="https://github.com/user-attachments/assets/334aeb74-0290-43e3-a052-f91a15dbf922" width="450"><br>
<img src="https://github.com/user-attachments/assets/dbe2ad96-43fd-4828-8880-aa11873cd315" width="450"><br>
<em>Şekil 3. Rota, irtifa ve yatay hız bilgileri</em>
</td>
<td align="center">
<img src="https://github.com/user-attachments/assets/01ddea3f-a40e-45b3-847b-796e8ca8dff1" width="450"><br>
<img src="https://github.com/user-attachments/assets/74841a30-8f47-4034-9c74-bcceb23b9463" width="450"><br>
<em>Şekil 5. Uçuş takibi duruş açıları animasyonu ve <br> roll, pitch ve yaw açı değişimleri</em>
</td>
</tr>
</table>
</div>

---

<div align="center">
<table>
  <thead>
    <tr>
      <th>Sensör</th>
      <th>Çıkış(lar)</th>
      <th>Oran (Hz)</th>
      <th>Gürültü (std)</th>
      <th>Bias/Drift</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>İvmeölçer</td>
      <td>a<sub>x</sub>, a<sub>y</sub>, a<sub>z</sub></td>
      <td>200</td>
      <td>σ<sub>a</sub> [m/s²]</td>
      <td>b<sub>a</sub> (ofset), RW</td>
    </tr>
    <tr>
      <td>Jiroskop</td>
      <td>ω<sub>x</sub>, ω<sub>y</sub>, ω<sub>z</sub></td>
      <td>200</td>
      <td>σ<sub>ω</sub> [rad/s]</td>
      <td>b<sub>ω</sub> (ofset), RW</td>
    </tr>
    <tr>
      <td>Manyetometre</td>
      <td>m<sub>x</sub>, m<sub>y</sub>, m<sub>z</sub></td>
      <td>50</td>
      <td>σ<sub>m</sub> [µT]</td>
      <td>-</td>
    </tr>
    <tr>
      <td>Barometre (altimetre)</td>
      <td>p (basınç) / h (irtifa)</td>
      <td>25</td>
      <td>σ<sub>p</sub> [Pa] / σ<sub>h</sub> [m]</td>
      <td>-</td>
    </tr>
    <tr>
      <td>GNSS</td>
      <td>p, v</td>
      <td>5</td>
      <td>σ<sub>pos</sub> [m], σ<sub>vel</sub> [m/s]</td>
      <td>-</td>
    </tr>
    <tr>
      <td>Pitot (hava hızı)</td>
      <td>q (din. bas.) / V (hava hızı)</td>
      <td>25</td>
      <td>σ<sub>q</sub> [Pa], σ<sub>V</sub> [m/s]</td>
      <td>-</td>
    </tr>
  </tbody>
</table>
<p style="margin-top:6px; font-size:12px;">
Tablo: Asenkron ölçüm modeli, sensörler ve çalışma frekansları.
</p>
</div>

## 15 State EKF’nin Durum Vektörü

<p align="center">
  <img width="1000" alt="15-state-ekf" src="https://github.com/user-attachments/assets/9e8db22c-f0c2-4ad2-a6bb-7566cea5dbce" /><br>
  <em>Şekil 6. Genişletilmiş Kalman Filtresi (EKF) için tanımlanan 15 durumlu vektör yapısı</em>
</p>

---

## GNSS Kesintisi – Barometre ve Pitot Devre Dışı

<p align="center">
  <img width="330" alt="position_no_baro_pitot" src="https://github.com/user-attachments/assets/51cf6c10-6b06-4670-9f90-36eb513a84f7">
  <img width="330" alt="velocity_no_baro_pitot" src="https://github.com/user-attachments/assets/97305094-bb5c-4607-be11-fdda57334bc9">
  <img width="330" alt="attitude_no_baro_pitot" src="https://github.com/user-attachments/assets/582a9055-a784-4e82-9474-e80c692cce5f"><br>
  <em>Şekil 7. GNSS kesintisi sırasında pozisyon, hız ve duruş kestirimlerinin barometre ve pitot sensörleri devre dışı iken elde edilen sonuçları</em>
</p>

---

## GNSS Kesintisi – Barometre ve Pitot Aktif

<p align="center">
  <img width="330" alt="position_baro_pitot_active" src="https://github.com/user-attachments/assets/5fedd403-dc77-4d02-baf8-a3a396c3a7e9">
  <img width="330" alt="velocity_baro_pitot_active" src="https://github.com/user-attachments/assets/5b6ee86d-ccf9-4c1b-b1e8-0f1cecf08541">
  <img width="330" alt="attitude_baro_pitot_active" src="https://github.com/user-attachments/assets/2340df70-1f57-4dc4-9c70-b7ae192c259f"><br>
  <em>Şekil 8. GNSS kesintisi sırasında pozisyon, hız ve duruş kestirimlerinin barometre ve pitot sensörleri aktif iken elde edilen sonuçları</em>
</p>


## Filtrelerin Karşılaştırılması - Duruş Açıları
<p align="center">
<img width="588" height="500" alt="image" src="https://github.com/user-attachments/assets/bc53979b-fe8b-4a4c-b055-3b097a057720" /><br>
  <em>Şekil 9. EKF, Madgwick ve Tamamlayıcı filtrelerin duruş açıları üzerinde karşılaştırılması</em>


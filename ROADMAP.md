# Yol Haritası

**Proje:** AI-Driven Autonomous Kinematic Synthesis and Parametric CAD Generation System

**Temel kural:** LLM sadece isteği anlar ve JSON'a çevirir, hiçbir ölçü üretmez.
Sayıları deterministik motor hesaplar, bağımsız simülasyon doğrular, Fusion çizer.

```
Cümle → LLM (Ollama, yerel) → Kinematik motor + kontroller → mechanism.json → Fusion 360
```

## A. Altyapı
- [x] **A1. Git.** Sürüm kontrolü, her adım kayıtlı
- [x] **A2. Testler.** `python -m pytest`, formüller ve simülasyon otomatik kontrol ediliyor
- [x] **A3. NumPy'siz motor.** Fusion'ın kendi Python'unda NumPy yok
- [x] **A4. Fusion paneli ve köprü.** Alt panelde sohbet + sonuç kartı; köprü ile Claude Fusion'da kod çalıştırıp ekran görüntüsü alabiliyor

## B. Dört çubuğu bitirmek
- [x] **B5. Q'yu cümleye bağlamak.** "Hızlı dönüşlü olsun" dendiğinde anlasın
- [x] **B6. LLM şemasını sıkılaştırmak.** Gereksiz "eksik bilgi" uyarılarını kesmek
- [x] **B7. Mukavemet.** Yük ve malzemeye göre kesit (burkulma, net kesit, eğilme, pim kayma/ezilme), emniyet katsayısı, tahrik torku
- [x] **B7b. Uzuv başına ayrı kesit.** Ortak pim çapı, uzuv başına genişlik/kalınlık; tek kesite göre %32 daha az malzeme
- [x] **B7c. Parametre kaydırıcıları.** Panelde anında hesaplanan kaydırıcılar, salınımda uygun bölge renkleri; Fusion'da "1 tur" ve sürekli döndürme
- [x] **B7d. Gerilme yığılması (Kt) ve sehim.** Roark Kt (kırılgan malzemede, eksenel gerilmeye); L/200 sehim sınırı (konsol eğilme / eksenel uzama)
- [x] **B7e. Fusion FEA ile doğrulama (niteliksel).** Al 6061 kol konsol, 49 N: kökte FEA n = 2–4, el hesabı n ≈ 3.6 ✓. Sim API önizlemede çalışmıyor, çözüm buluta gitti, sonuç sayısal okunamıyor
- [x] **B7f. Yerel, otomatik FEA.** gmsh + scikit-fem (tamamen yerel); kol ve krank için FEA ile el hesabı karşılaştırması; panelde "FEA" düğmesi (Fusion içinde deneme bekliyor)
- [x] **B7g. Göbek esnekliği.** Model δ_göbek = P·L²/(C·E·t·r²), C FEA ile kalibre (21 geometri, C = 6–9, L'den bağımsız); güvenli taraf için C = 6
- [x] **B7h. Mil ve pim çaplarını ayırmak.** O2/O4 çelik mil (burulma + kayma kontrolü), A/B pim; Fusion her uçta doğru deliği çiziyor. PETG 5 kg: biyel 25→13 mm, hacim −%12
- [x] **B8a. Yerel göbek (boss).** Delikler kendi göbeğinde (D = 2.5·d), gövde kontrollerin gerektirdiği kadar ince; göbek ve gövde kesitleri ayrı kontrol; Fusion'da dolu extrude + delik kesme. PETG hacim −%29, Al −%59
- [x] **B8b. Katman çakışması.** Ara katmandan geçen milin, hareketli uzuvlardan (gövde + göbek) çevrim boyunca en az 1 mm uzak kalması; gerekirse katman sırası değişir, olmazsa tasarım reddedilir
- [ ] **B8c. Pimler ve boşluk (print-in-place).** Pim/başlık geometrisi, radyal ve eksenel boşluk (baskı zamanı, D12 ile birlikte)
- [x] **B9. Tam tur atmayan dört çubuklar.** Freudenstein fonksiyon üreteci (3 Chebyshev hassas noktası, θ₀/φ₀ taraması, yapısal hata ≤ 1°); mukavemet + çakışmadan geçen ilk aday; LLM "rocker_rocker"; panelde "Giriş hareketi"; Fusion'da mafsal sınırı ve ileri-geri salınım

## C. Yeni modüller
- [x] **C0. Modüle özel kaydırıcılar.** pipeline.MODULES (ad, birim, aralık); panel "Tür" seçicisi ve kaydırıcıları tanımdan kuruyor; sohbetten gelen tasarım doğru modüle geçiyor
- [x] **C9. Düz dişli.** Lewis eğilme, kavrama oranı, alttan kesme, Hertz (bilgi), çelik mil ve göbek; Fusion'da evolvent dişler, dairesel çoklama, motion link (diş kesişimi 0, oran doğrulandı)
- [x] **C10. Helis ve konik yay.** Shigley: Bergsträsser, S_ut = A/dᵐ, tam sıkışmada akma yok, burkulma; konik k = G·d⁴/(16·Nₐ(R₁+R₂)(R₁²+R₂²)); Fusion'da helis sweep + taşlanmış uçlar
- [x] **L1. İki aşamalı LLM.** Yönlendirici (tür) + modüle özel kısa çıkarıcı; tek uzun istem 9B modelde ayrıntı kaçırıyordu; llm_eval 16/16
- [x] **C11a. Kam.** Sikloidal / harmonik / 3-4-5; basınç açısı ≤ 30°, alttan kesme yok, ρ ≥ 2·R_makara; en küçük temel daire; Fusion'da kam + makaralı itici + s(θ) animasyonu (8 açıda temas boşluğu 0, kesişim 0)
- [ ] **C11b. Mil.** Shigley 7. bölüm, DE-Goodman yorulma

## D. Dış dünya
- [ ] **D12. Fiziksel baskı** (arkadaşın yazıcısı)
- [ ] **D13. README ve demo videosu** (portföy)
- [ ] **D14. İleride:** FreeCAD/AutoCAD desteği, "serbest mod" (doğrulamasız, etiketli)

## Bulgular (rapor için)
- **Q = 1** için bağıntı `c = a / sin(ψ/2)`. Kompaktlık ile iletim açısı arasında ödünleşim var.
  Biyel uzadıkça μ_min, 90 − ψ/2 değerine yaklaşır. Bu yüzden ψ > 90° iken 45° hedefi sağlanamaz.
- **Hızlı dönüş (Q > 1)** iletim açısını **kötüleştiriyor.** Örneğin ψ = 90° için μ_min değerleri
  Q = 1.1 / 1.2 / 1.3 iken 34° / 30° / 27°. Büyük salınım için çözüm dişli kademesi ya da altı uzuvlu mekanizma.
- **Sentez tek başına güvenilir değil.** İlk hızlı dönüş sürümü "110° ✓" dedi, simülasyon 47.7° ölçtü
  (uç konumlar farklı montaj kollarına düşmüştü). Bu yüzden her tasarım bağımsız simülasyonla doğrulanıyor.
- **Üçlü doğrulama:** 35 mm krank, 55° salınım için formül (Norton), bağımsız simülasyon (55.00°) ve
  Fusion'ın montaj çözücüsü (krank mafsalı köprüyle döndürüldü, 54.99°) aynı sonucu veriyor.
- **Biyel kuvveti F_AB = F / sin μ:** İletim açısı düştükçe biyel ve pim yükü artıyor (μ = 45° → 1.41F).
  Mukavemet hesabı, iletim açısı kontrolünün neden gerekli olduğunu sayısal olarak gösteriyor.
- **Yanlış sezgi, doğru motor:** "Kısa krank daha küçük kesit almalı" varsayımı yanlış çıktı. Krank tahrik torkuyla
  eğilmeye çalışıyor (genişlik ister), sabit uzuv ise sadece eksenel yük taşıyor (burkulma, kalınlık ister).
  5 kg / PETG örneği: Krank 21×3, Biyel 13×3, Kol 24×3, Sabit 11×4 mm.
- **Alüminyumda rijitlik belirleyici:** 5 kg örneğinde kolda gerilme kullanımı %57 iken sehim kullanımı %94.
  Metalde parçayı kırılma değil esneme sınırı boyutlandırıyor. PETG'de sehim sınırı kolu 24'ten 29 mm genişliğe çıkardı.
- **Yerel FEA (gmsh + scikit-fem, P2 tetra) doğrulaması:** Orta kesit gerilmesi el hesabıyla %0.1–5 uyumlu
  (kiriş teorisi doğrulandı). Sehim ise ideal ankastre varsayımından büyük: Al 9×3 kol +%13, PETG 29×3 kol +%311.
  Sebep: geniş ve rijit uzuv, küçük (Ø4) delik çevresinde dönüyor (göbek esnekliği). Ağ yakınsaması kontrol edildi.
- **Göbek modeli (FEA kalibrasyonu):** Delikten tutulan uzvun dönme rijitliği k = C·E·t·r². 21 FEA çözümünde
  C, uzunluktan bağımsız (±%3, L² ölçeklemesi doğrulandı) ve d/w ≤ 0.3 için 5.96–9.17. Güvenli taraf için C = 6.
  Etkisi: 40 mm, 29 mm genişlik, Ø3 pim uzuvda göbek sehimi kiriş eğilmesinin 11 katı.
- **Gizli güvensizlik yakalandı:** Mil burulma kontrolü eklenince, önceki alüminyum tasarımdaki Ø3 ortak pimin
  2.94 N·m çıkış torkunda τ ≈ 555 MPa'ya çıkacağı görüldü (çelik bile kopar). Tork taşıyan mafsal ile serbest pim
  ayrı düşünülmeli. PETG örneği (mil Ø12, pim Ø4): FEA ile model sehimi %24–35 güvenli tarafta, gerilme %6–11 uyumlu.
- **Göbekli geometri FEA'sı:** Model sehimi FEA'nın %24–54 üstünde (göbek, mil çevresini rijitleştiriyor; model güvenli tarafta),
  gerilme %1–12 uyumlu. Kalın mil sadece kendi göbeğini büyütüyor: PETG biyel ve sabit uzuv 6 mm gövdeye indi.
- **Fonksiyon üreteci (Freudenstein):** 40 mm giriş kolu, 60° giriş → 90° çıkış: simülasyon 89.0°, Fusion çözücüsü 89.4°
  (yapısal hata < 1°, hassas noktalarda sıfır). Çıkan mekanizmalar çoğunlukla Grashof değil (üçlü kol): giriş tam tur atmıyor.
  "En kompakt" seçimi göbeklerin sığmayacağı kısa uzuvlar üretti; seçim ölçütü "dengeli oran" yapıldı.
- **Dişli doğrulaması (Fusion):** Oran 3 / 2 N·m / PLA → m = 2, z = 18/54, b = 16 mm. Beş dönüş konumunda pinyon ile çark
  arasındaki kesişim hacmi 0 mm³ (evolvent fazı doğru); pinyon 90° → çark −30° (oran ve yön doğru).
- **Yay doğrulaması (Fusion):** 100 N / 20 mm → Ø1.6 tel, D = 9.6 mm, 15 sarım; Fusion'da boy 50.2 mm (L₀ = 50.2), dış çap 11.2 mm.
  Konik yay (50 N / 15 mm, D₂/D₁ = 0.5): boy 30.71 mm, dış çap 16.1 mm, tasarımla aynı. Konik yay sabiti formülü
  sarım sarım sayısal integralle doğrulandı (test).
- **Tek uzun LLM istemi ölçeklenmiyor:** Yay alanları eklenince eski bir dört çubuk cümlesi bozuldu (15/16).
  İki aşamalı yapı (yönlendirici + modül çıkarıcısı) hem 16/16 verdi hem de istemleri kısalttığı için ~6 s'de kaldı.
- **Kam doğrulaması (Fusion):** 20 mm kalkma, 120/60/120° sikloidal → R_b hesaplandı; kam 8 farklı açıya döndürülüp itici
  s(θ) tablosundan konumlandırıldığında makara-kam arası en kısa mesafe 0.000 mm ve kesişim 0.00 mm³ (tam temas).
  Ters yönde döndürünce 1.8–15.5 mm boşluk / 640 mm³ kesişim: test ayırt edici. Yolda bulunan hata: kayar mafsalda
  özel yön (kök ekseni) verilince Fusion onu yok sayıp z yönünde kaydırıyordu; itici hiç hareket etmiyor görünüyordu.

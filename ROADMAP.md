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
- [ ] **B8. Pimler ve boşluk.** Print-in-place baskıya hazırlık; mil çevresinde yerel göbek (boss) ile uzvun geri kalanını inceltmek

## C. Yeni modüller
- [ ] **C0. Modüle özel kaydırıcılar.** Her modül kendi parametrelerini (ad, birim, aralık) tanımlar, panel kaydırıcıları otomatik kurar (yay seçiliyse yay kaydırıcıları). İlk yeni modülle birlikte
- [ ] **C9. Düz dişli.** Dört çubuk + dişli kademesi ile büyük salınım
- [ ] **C10. Helis ve konik yay**
- [ ] **C11. Mil, kam**

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

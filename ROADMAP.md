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
- [ ] **B7. Mukavemet.** Yük ve malzemeye göre uzuv kesiti, emniyet katsayısı
- [ ] **B8. Pimler ve boşluk.** Print-in-place baskıya hazırlık

## C. Yeni modüller
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

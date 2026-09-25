# Demo videosu: çekim senaryosu (~2 dakika)

**Kayıt:** Windows'ta `Win + Alt + R` (Xbox Game Bar), Fusion penceresini kaydeder. Tekrar bas: durur.
Kayıttan önce Ollama açık olsun ve bir cümle deneyip modeli ısıt (ilk istek yavaş olur).
Panel serbest pencere; Fusion'un altına yerleştir, model görünümü üstte kalsın.

| # | Süre | Ne yapılır | Ekranda görünen / söylenecek (altyazı) |
|---|---|---|---|
| 1 | 0:00–0:10 | Boş Fusion + panel | "Mekanizmayı cümleyle tarif et; yerel yapay zekâ anlar, hesapları Python yapar, Fusion çizer." |
| 2 | 0:10–0:35 | Sohbete yaz: `30 mm krank, 60° salınım, uçta 5 kg, PETG` → Gönder | Sonuç kartı: uzuv boyları, iletim açısı grafiği, ✓ kontroller. Fusion'da renkli dört çubuk. |
| 3 | 0:35–0:45 | **▶ Sürekli** | Mekanizma döner. "Salınım Fusion'ın kendi çözücüsüyle de 55/55° doğrulandı." |
| 4 | 0:45–1:00 | Salınım kaydırıcısını 60 → 120'ye çek | Kırmızı bölgeye girince ✗ ve neden (iletim açısı). "Olmayan tasarımı çizmiyor, nedenini söylüyor." |
| 5 | 1:00–1:15 | Yaz: `4 cm kolu 45° çevirince diğer kol 120° dönsün, PETG, 2 kg` → çiz → **⟳ 1 tur** | Sınırlı açıda ileri-geri salınan dört çubuk. |
| 6 | 1:15–1:25 | Tür: **Dişli** → çiz → Sürekli | Dişler birbirine girmeden döner. |
| 7 | 1:25–1:35 | Tür: **Yay**, konik oranı 0.5, görünüm "Plakalarla" → çiz → 1 tur | Konik yay sıkışıp açılır. |
| 8 | 1:35–1:45 | Tür: **Kam** → çiz → Sürekli | İtici kam profilini takip eder. |
| 9 | 1:45–1:55 | Tür: **Mil**, gerilme yığılması: kama yuvası → segman kanalı | Çap büyür (K_t 2.14 → 5.0). Çiz: kama yuvalı mil. |
| 10 | 1:55–2:05 | Terminalde `python -m pytest` (isteğe bağlı) | "113 test geçti." Son kare: README'deki görsel tablosu. |

**İpuçları**
- Her sahneden önce görünümü "Home" (ev simgesi) yap; kamera sabit kalsın.
- Uzun bekleme olursa videoda kes (LLM ~5 s).
- Ses yerine altyazı daha kolay: kaydı Clipchamp'e (Windows'ta hazır) at, metinleri tablodaki gibi ekle.

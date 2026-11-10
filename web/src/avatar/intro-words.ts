// 2026-11-09 audio: local faster-whisper small/int8; spelling corrected to supplied script.
// Intra-word vowel times are interpolated, not phoneme-forced alignment.
export const INTRO_WORDS = [
  {
    "start": 1.08,
    "end": 1.94,
    "word": "안녕하세요."
  },
  {
    "start": 2.2,
    "end": 2.48,
    "word": "제"
  },
  {
    "start": 2.48,
    "end": 2.94,
    "word": "이름은"
  },
  {
    "start": 2.94,
    "end": 3.72,
    "word": "막내."
  },
  {
    "start": 4.0,
    "end": 4.34,
    "word": "Mobile"
  },
  {
    "start": 4.34,
    "end": 4.9,
    "word": "Assistant"
  },
  {
    "start": 4.9,
    "end": 5.5,
    "word": "for"
  },
  {
    "start": 5.5,
    "end": 5.88,
    "word": "Kin's"
  },
  {
    "start": 6.12,
    "end": 6.38,
    "word": "Needs"
  },
  {
    "start": 6.38,
    "end": 6.58,
    "word": "And"
  },
  {
    "start": 6.58,
    "end": 7.14,
    "word": "Errands"
  },
  {
    "start": 7.14,
    "end": 7.88,
    "word": "약자예요."
  },
  {
    "start": 8.5,
    "end": 9.46,
    "word": "이름은"
  },
  {
    "start": 9.46,
    "end": 10.08,
    "word": "거창하게"
  },
  {
    "start": 10.08,
    "end": 10.9,
    "word": "지어놨지만,"
  },
  {
    "start": 11.42,
    "end": 11.82,
    "word": "쉽게"
  },
  {
    "start": 11.82,
    "end": 12.3,
    "word": "말하면"
  },
  {
    "start": 12.3,
    "end": 12.78,
    "word": "가족"
  },
  {
    "start": 12.78,
    "end": 13.32,
    "word": "곁에서"
  },
  {
    "start": 13.32,
    "end": 13.78,
    "word": "필요한"
  },
  {
    "start": 13.78,
    "end": 14.02,
    "word": "일을"
  },
  {
    "start": 14.02,
    "end": 14.34,
    "word": "돕는"
  },
  {
    "start": 14.34,
    "end": 14.82,
    "word": "막내"
  },
  {
    "start": 14.82,
    "end": 15.46,
    "word": "포지션이죠."
  },
  {
    "start": 16.2,
    "end": 16.52,
    "word": "몸을"
  },
  {
    "start": 16.52,
    "end": 17.04,
    "word": "움직이기"
  },
  {
    "start": 17.04,
    "end": 17.26,
    "word": "힘들"
  },
  {
    "start": 17.26,
    "end": 17.66,
    "word": "때"
  },
  {
    "start": 17.66,
    "end": 18.6,
    "word": "물병이나"
  },
  {
    "start": 18.6,
    "end": 19.3,
    "word": "안경처럼"
  },
  {
    "start": 19.3,
    "end": 19.7,
    "word": "꼭"
  },
  {
    "start": 19.7,
    "end": 20.18,
    "word": "필요한"
  },
  {
    "start": 20.18,
    "end": 20.62,
    "word": "물건을"
  },
  {
    "start": 20.62,
    "end": 21.02,
    "word": "가져다"
  },
  {
    "start": 21.02,
    "end": 21.28,
    "word": "주는"
  },
  {
    "start": 21.28,
    "end": 22.0,
    "word": "작은"
  },
  {
    "start": 22.0,
    "end": 22.54,
    "word": "심부름"
  },
  {
    "start": 22.54,
    "end": 23.12,
    "word": "로봇이에요."
  },
  {
    "start": 24.2,
    "end": 25.18,
    "word": "평소에는"
  },
  {
    "start": 25.18,
    "end": 25.82,
    "word": "얌전히"
  },
  {
    "start": 25.82,
    "end": 26.42,
    "word": "부탁하신"
  },
  {
    "start": 26.42,
    "end": 26.68,
    "word": "일을"
  },
  {
    "start": 26.68,
    "end": 27.58,
    "word": "도와드리지만,"
  },
  {
    "start": 28.04,
    "end": 29.2,
    "word": "마냥"
  },
  {
    "start": 29.2,
    "end": 29.68,
    "word": "시키는"
  },
  {
    "start": 29.68,
    "end": 30.02,
    "word": "대로만"
  },
  {
    "start": 30.02,
    "end": 30.5,
    "word": "움직이는"
  },
  {
    "start": 30.5,
    "end": 30.68,
    "word": "건"
  },
  {
    "start": 30.68,
    "end": 31.44,
    "word": "아니거든요?"
  },
  {
    "start": 32.04,
    "end": 32.54,
    "word": "주변도"
  },
  {
    "start": 32.54,
    "end": 33.2,
    "word": "살피고,"
  },
  {
    "start": 33.32,
    "end": 33.56,
    "word": "누가"
  },
  {
    "start": 33.56,
    "end": 34.1,
    "word": "가까이"
  },
  {
    "start": 34.1,
    "end": 34.38,
    "word": "오면"
  },
  {
    "start": 34.38,
    "end": 34.76,
    "word": "잠깐"
  },
  {
    "start": 34.76,
    "end": 35.5,
    "word": "멈추고요."
  },
  {
    "start": 35.98,
    "end": 36.64,
    "word": "작아도"
  },
  {
    "start": 36.64,
    "end": 37.06,
    "word": "제법"
  },
  {
    "start": 37.06,
    "end": 37.66,
    "word": "신중하죠?"
  },
  {
    "start": 38.42,
    "end": 38.7,
    "word": "앞으로"
  },
  {
    "start": 38.7,
    "end": 39.2,
    "word": "필요한"
  },
  {
    "start": 39.2,
    "end": 39.68,
    "word": "순간에"
  },
  {
    "start": 39.68,
    "end": 40.02,
    "word": "저"
  },
  {
    "start": 40.02,
    "end": 40.5,
    "word": "막내를"
  },
  {
    "start": 40.5,
    "end": 41.04,
    "word": "불러주세요."
  },
  {
    "start": 41.54,
    "end": 41.76,
    "word": "잘"
  },
  {
    "start": 41.76,
    "end": 42.4,
    "word": "부탁드려요."
  }
];

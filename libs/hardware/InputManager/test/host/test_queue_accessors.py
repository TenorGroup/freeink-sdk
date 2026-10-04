"""Compile the SDK queue/accessor methods unchanged, with RAM queue and touch-state fixtures.

This isolates sync/async consumption and release suppression from hardware drivers.
It does not simulate controller reads or prove physical GT911 timing.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

source_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / 'src/InputManager.cpp'
source = source_path.read_text()
methods = ['popMultiTouchSwipe', 'wasMultiTouchSwipe', 'normalizeTouchPoint', 'suppressTouchContact',
           'wasTouchTap', 'wasSwipe', 'wasTouchReleased']
implementations = []
for name in methods:
    anchor = source.index('InputManager::' + name + '(')
    begin = source.rfind('\n', 0, anchor) + 1
    end = source.index('\n}\n', anchor) + 3
    implementations.append(source[begin:end])
fixture = r'''
#include <cassert>
#include <cstdint>
#include <deque>
#include <cmath>
#define FREEINK_CAP_TOUCH 1
constexpr int pdTRUE=1;
constexpr int TOUCH_SWIPE_MIN_PX=60, TOUCH_SLOW_DRAG_PX=60, TOUCH_SWIPE_MAX_MS=700;
int absInt(int v) { return v<0?-v:v; }
namespace BoardConfig {
struct Touch { uint16_t rawMaxX=800,rawMinX=0,rawMaxY=480,rawMinY=0; };
struct Profile { Touch touch; };
Profile ACTIVE;
}
struct QueuedMultiTouchSwipe { uint16_t startX,startY,endX,endY,durationMs; uint8_t contactCount; };
using Queue = std::deque<QueuedMultiTouchSwipe>;
int xQueueReceive(void* q, QueuedMultiTouchSwipe* out, int) {
  auto& queue=*static_cast<Queue*>(q);
  if(queue.empty()) return 0;
  *out=queue.front(); queue.pop_front(); return pdTRUE;
}
void xQueueReset(void* q) { static_cast<Queue*>(q)->clear(); }
struct Point { uint16_t x=0,y=0; };
class InputManager {
public:
  void* _asyncMultiTouchSwipeQueue=nullptr;
  void* _asyncMultiTouchRotationQueue=nullptr;
  void* _asyncMultiTouchPinchQueue=nullptr;
  bool multiTouchSwipeEvent=false,touchSuppressed=false,touchPressed=false,touchReleasedEvent=false;
  bool touchMultiContactSequence=false,touchMovedBeyondTapReleaseSlop=false;
  uint8_t multiTouchSwipeContactCount=2;
  uint16_t multiTouchSwipeStartX=400,multiTouchSwipeStartY=300,multiTouchSwipeEndX=400,multiTouchSwipeEndY=180;
  unsigned long multiTouchSwipeDurationMs=250,lastTouchHeldDurationMs=250;
  Point touchDownPoint{400,300},touchUpPoint{400,180};
  void cancelMultiTouchGesture() { multiTouchSwipeEvent=false; }
  bool popMultiTouchSwipe(uint8_t&,float&,float&,float&,float&,unsigned long&);
  bool wasMultiTouchSwipe(uint8_t&,float&,float&,float&,float&,unsigned long&) const;
  void normalizeTouchPoint(uint16_t,uint16_t,float&,float&) const;
  void suppressTouchContact();
  bool wasTouchTap(float&,float&) const;
  bool wasSwipe(float&,float&,float&,float&) const;
  bool wasTouchReleased() const;
};
'''
test = r'''
int main() {
  InputManager input;
  uint8_t count=0;
  float sx=0,sy=0,ex=0,ey=0;
  unsigned long ms=0;
  input.multiTouchSwipeEvent=true;
  input.touchReleasedEvent=true;
  input.touchMultiContactSequence=true;
  assert(input.popMultiTouchSwipe(count,sx,sy,ex,ey,ms));
  assert(count==2 && sx==.5f && ex==.5f && sy>.6f && ey<.4f && ms==250);
  assert(!input.popMultiTouchSwipe(count,sx,sy,ex,ey,ms));
  assert(!input.wasTouchTap(sx,sy));
  assert(!input.wasSwipe(sx,sy,ex,ey));
  assert(input.wasTouchReleased());
  input.suppressTouchContact();
  assert(input.touchSuppressed && !input.wasTouchTap(sx,sy) && !input.wasSwipe(sx,sy,ex,ey));
  // Preserve queued FIFO semantics when async polling is enabled.
  Queue queue{{10,20,30,40,50,2},{50,60,70,80,90,3}};
  input._asyncMultiTouchSwipeQueue=&queue;
  input.touchSuppressed=false;
  assert(input.popMultiTouchSwipe(count,sx,sy,ex,ey,ms) && count==2 && ms==50 && queue.size()==1);
  assert(input.popMultiTouchSwipe(count,sx,sy,ex,ey,ms) && count==3 && ms==90 && queue.empty());
  assert(!input.popMultiTouchSwipe(count,sx,sy,ex,ey,ms));
  queue.push_back({10,20,30,40,50,2});
  input.suppressTouchContact();
  assert(queue.empty() && !input.popMultiTouchSwipe(count,sx,sy,ex,ey,ms));
}
'''
with tempfile.TemporaryDirectory(prefix='freeink-queue-accessors-') as tmp:
    cpp=Path(tmp)/'accessors.cpp'
    binary=Path(tmp)/'accessors'
    cpp.write_text(fixture+'\n'.join(implementations)+test)
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('GREEN: SDK actual sync pop once, async FIFO, multi-release tap/swipe suppression')

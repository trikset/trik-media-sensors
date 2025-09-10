#ifndef TRIK_SENSORS_HSV_RANGE_DETECTOR_HPP_OBJECT_
#define TRIK_SENSORS_HSV_RANGE_DETECTOR_HPP_OBJECT_

#ifndef __cplusplus
#error C++-only header
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <c6x.h>
#include <cassert>
#include <cmath>



/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace sensors /* **** **** **** **** **** */ {

class HsvRangeDetectorObject
{
  private:
  //real clasters params
    static const int cstrs_max_num = 32; // 256/8
    static const int pos_shift = 3; // 8 == 2^3

    /*
    //test clasters params
    static const int cstrs_max_num = 16; // 256/16
    static const int pos_shift = 4; // 16 == 2^4
    */
    static int32_t s_hs_clasters[cstrs_max_num][cstrs_max_num];
    int width;
    int height;
    
    //positive image part bounds
    int pos_l;
    int pos_r;
    int pos_t;
    int pos_b;

    //negative image part bounds
    int neg_l;
    int neg_r;
    int neg_t;
    int neg_b;

    //penalty coeffs
    static const int K0 = 2;
    static const int K1 = 1; 
    static const int K2 = 2; 

    const double T_end = 0.0005;

    const double e = 2.718281828;
    const double lambda = 0.76;

    union U_Hsv8x3
    {
        struct {
          uint8_t h;
          uint8_t s;
          uint8_t v;
          uint8_t none;
        } parts;
        uint32_t whole;
    };

    int getIncrement(int _val, int _min, int _max, double _t)
    {
        assert(_min <= _max);
        if(_min == _max)
        {
            return _min;
        }
        else
        {
            int res = 0;
            double base = 1 + 1/_t;
            double alpha = rand()/static_cast<double>(RAND_MAX);
            double degree = 2 * alpha - 1;

            res = _val + ((pow(base, degree) - 1) * _t)*static_cast<double>(_max - _min);

            if ((_min <= res) && (res < _max))
                return res;
            else
                return getIncrement(_val, _min, _max, _t);
        }
    }


    int truncateHue(int _val)
    {
        int res = _val % cstrs_max_num;
        if (res < 0)
            res += cstrs_max_num;
        return res;
    }

    uint64_t m_foo(int h1, int h2, int s1, int s2)
    {
        int64_t res = 0;
        int32_t val = 0;

        if (h1 <= h2)
        {

            for(int h_pos = h1; h_pos <= h2; h_pos++)
            {
                for(int s_pos = s1; s_pos <= s2; s_pos++)
                {
                    val = s_hs_clasters[h_pos][s_pos];
                    res += val != 0 ? val : -K0;

                    //res += s_hs_clasters[h_pos][s_pos];
                }
            }

        }
        else // h1 > h2
        {
            for(int h_pos = h1; h_pos < cstrs_max_num; h_pos++)
            {
                for(int s_pos = s1; s_pos <= s2; s_pos++)
                {
                    val = s_hs_clasters[h_pos][s_pos];
                    res += val != 0 ? val : -K0;

                    //res += s_hs_clasters[h_pos][s_pos];
                }
            }
            for(int h_pos = 0; h_pos <= h2; h_pos++)
            {
                for(int s_pos = s1; s_pos <= s2; s_pos++)
                {
                    val = s_hs_clasters[h_pos][s_pos];
                    res += val != 0 ? val : -K0;

                }
            }
        }

        return res;
    }

  public:
    HsvRangeDetectorObject(int _imgWidth, int _imgHeight, int _detectZoneScale)
    {
      width = _imgWidth;
      height = _imgHeight;

      int hHeight = height/2;
      int hWidth = width/2;
      int step = height/_detectZoneScale;

      pos_l = hWidth - step;
      pos_r = hWidth + step;
      pos_t = hHeight - step;
      pos_b = hHeight + step;

      //negative image part bounds
      neg_l = hWidth - 2*step;
      neg_r = hWidth + 2*step;
      neg_t = hHeight - 2*step;
      neg_b = hHeight + 2*step;
    }

    void detect(uint16_t& _hFrom, uint16_t& _hTo, uint8_t& _sFrom, uint8_t& _sTo, uint8_t& _vFrom, uint8_t& _vTo, uint64_t* _rgb888hsv) 
    {
    //initialize stuff
      srand(time(NULL));
      const uint64_t* restrict img = _rgb888hsv;
      int h1;
      int h2;
      int s1;
      int s2;

    //initialize clasters
      memset(s_hs_clasters, 0, cstrs_max_num*cstrs_max_num*sizeof(int32_t));

    //initialize variables for claster with highest occurrence
      int h_max_pos = 0;
      int s_max_pos = 0;
      int max_value = 0;

    //clasterize image
      U_Hsv8x3 pixel;
      int h_pos = 0;
      int s_pos = 0;
      
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          pixel.whole = _loll(*(img)++);
          h_pos = (pixel.parts.h >> pos_shift);
          s_pos = (pixel.parts.s >> pos_shift);

          //positive part of image
          if(pos_l < col && col < pos_r && pos_t < row && row < pos_b)
          {
            s_hs_clasters[h_pos][s_pos]+=K1;

            //remember claster with highest positive occurrence
            if (s_hs_clasters[h_pos][s_pos] > max_value)
            {
                max_value = s_hs_clasters[h_pos][s_pos];
                h_max_pos = h_pos;
                s_max_pos = s_pos;
            }

          } //negative part of image
          else if(neg_r < col || col < neg_l || neg_b < row || row < neg_t) //wrong condition!!
          {
            s_hs_clasters[h_pos][s_pos]-=K2;
          }
        }
      }

    //algorithm

      //initial hsv range
      h1 = h_max_pos;
      h2 = h_max_pos;
      s1 = s_max_pos;
      s2 = s_max_pos;

      int64_t L = m_foo(h1, h2, s1, s2);

      double T = 150;

      int h1New;
      int h2New;
      int s1New;
      int s2New;

      while(/*Th > T_end || Ts > T_end || Tv > T_end*/ T > T_end)
      {
#pragma MUST_ITERATE(200, ,200)
        for(int i = 0; i < 200; i++)
        {
          h1New = truncateHue(getIncrement(h1, 0, cstrs_max_num, T));
          h2New = truncateHue(getIncrement(h2, 0, cstrs_max_num, T));
          s1New = getIncrement(s1, 0, s_max_pos, T);
          s2New = getIncrement(s2, s_max_pos, cstrs_max_num, T);

          int64_t LNew = m_foo(h1New, h2New, s1New, s2New);

          if(L < LNew || (rand()/(double)RAND_MAX) <= pow(e,-(L-LNew)/T))
          {
            h1 = h1New;
            h2 = h2New;
            s1 = s1New;
            s2 = s2New;

            L = LNew;
          }
        }
        T*=lambda;
      }

      h1 = (h1 << pos_shift)*1.4f;
      h2 = (((h2+1) << pos_shift) - 1)*1.4f;

      s1 = (s1 << pos_shift)*0.39f;
      s2 = (((s2+1) << pos_shift))*0.39f;

      uint8_t _h, _hTol, _s, _sTol;
      if (h1 <= h2) 
      {
        _h    = (h2 + h1) / 2;
        _hTol = (h2 - h1) / 2;
      }
      else
      {
        float hue = (h2 - (360.0f - h1)) / 2;
        float hueTolerance = (h2 + (360.0f - h1)) / 2;
        _h = hue >= 0 ? hue : (hue + 360);
        _hTol = hueTolerance;
      }

      _s = (s2 + s1) / 2;
      _sTol = (s2 - s1) / 2 + 2;

      _hFrom = makeValueWrap(_h, -_hTol, 0, 359);
      _hTo = makeValueWrap(_h, +_hTol, 0, 359);

      _sFrom = makeValueRange(_s, -_sTol, 0, 100);
      _sTo = makeValueRange(_s, +_sTol, 0, 100);

      _vFrom = 0;
      _vTo = 100;
    }

};

int32_t restrict HsvRangeDetectorObject::s_hs_clasters[HsvRangeDetectorObject::cstrs_max_num][HsvRangeDetectorObject::cstrs_max_num];


} /* **** **** **** **** **** * namespace sensors * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */


#endif // !TRIK_SENSORS_HSV_RANGE_DETECTOR_HPP_OBJECT_

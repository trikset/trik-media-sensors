/* ======================================================================== *
 * IMGLIB -- TI Image and Video Processing Library                          *
 *                                                                          *
 *                                                                          *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/   *
 *                                                                          *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 * ======================================================================== */

/* ======================================================================== */
/*  NAME                                                                    */
/*      IMG_sobel_5x5_16s                                                   */
/*                                                                          */
/*  USAGE                                                                   */
/*      This routine is C-callable and can be called as:                    */
/*                                                                          */
/*      void IMG_sobel_5x5_16s                                              */
/*      (                                                                   */
/*          const short restrict  *in,     // Input image data  //          */
/*                short restrict *out,     // Output image data //          */
/*                short          cols,     // image columns     //          */
/*                short          rows      // Image rows        //          */
/*      )                                                                   */
/*                                                                          */
/*      The sobel filter is applied to the input image.  The input image    */
/*      dimensions are given by the arguments 'cols' and 'rows'.  The       */
/*      output image is 'cols' pixels wide and 'rows - 4' pixels tall.      */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      To see how the implementation is going to work on the input         */
/*      buffer, lets imagine we have the following input buffer:            */
/*      lets imagine we have the following input buffer:                    */
/*                                                                          */
/*              yyyyyyyyyyyyyyyy                                            */
/*              yyyyyyyyyyyyyyyy                                            */
/*              yyxxxxxxxxxxxxyy                                            */
/*              yyxxxxxxxxxxxxyy                                            */
/*              yyxxxxxxxxxxxxyy                                            */
/*              yyxxxxxxxxxxxxyy                                            */
/*              yyyyyyyyyyyyyyyy                                            */
/*              yyyyyyyyyyyyyyyy                                            */
/*                                                                          */
/*      The output buffer would be:                                         */
/*                                                                          */
/*              ttXXXXXXXXXXXXzz                                            */
/*              zzXXXXXXXXXXXXzz                                            */
/*              zzXXXXXXXXXXXXzz                                            */
/*              zzXXXXXXXXXXXXtt                                            */
/*                                                                          */
/*      Where:                                                              */
/*                                                                          */
/*          X = sobel(x)    The algorithm is applied to that pixel.         */
/*                          The correct output is obtained, the data        */
/*                          around the pixels we work on is used            */
/*                                                                          */
/*          t               Whatever was in the output buffer in that       */
/*                          position is kept there.                         */
/*                                                                          */
/*          z = sobel(y)    The algorithm is applied to that pixel.         */
/*                          The output is not meaningful, because the       */
/*                          necessary data to process the pixel is not      */
/*                          available.  This is because for each output     */
/*                          pixel we need input pixels from the right and   */
/*                          from the left of the output pixel.  But this    */
/*                          data doesn't exist.                             */
/*                                                                          */
/*      This means that we will only process (rows-4) lines.  Then, we      */
/*      will process all the pixels inside each line. Even though the       */
/*      results for the first and last pixels in each line will not         */
/*      be relevant, it makes the control much simpler and ends up          */
/*      saving cycles.                                                      */
/*                                                                          */
/*      Also the fist two pixels in the first processed line and the        */
/*      last two pixel in the last processed line will not be computed.     */
/*      It is not necessary, since the results would be bogus, and          */
/*      not computing them saves some time.                                 */
/*                                                                          */
/*      The following horizontal and vertical masks that are                */
/*      applied to the input buffer to obtain one output pixel.             */
/*                                                                          */
/*          Horizontal:                                                     */
/*              -1  -4  -6  -4  -1                                          */
/*              -2  -8 -12  -8  -2                                          */
/*               0   0   0   0   0                                          */
/*               2   8  12   8   2                                          */
/*               1   4   6   4   1                                          */
/*                                                                          */
/*          Vertical:                                                       */
/*               1   2   0  -2  -1                                          */
/*               4   8   0  -8  -4                                          */
/*               6  12   0 -12  -6                                          */
/*               4   8   0  -8  -4                                          */
/*               1   2   0  -2  -1                                          */
/*                                                                          */
/*                                                                          */
/*      The natural C implementation has no restrictions. The Optimized     */
/*      and IntrinsiC codes have restrictions as noted in the ASSUMPTIONS   */
/*      bellow.                                                             */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*      1. cols :     'cols' >  5 and multiple of 2.                        */
/*      2. rows :     'rows' >= 5.                                          */
/*      3. cols * (rows - 4) - 4 should be greater than or equal to 2.      */
/*      4. Input and output arrays should not overlap.                      */
/*      5. Input and Output arrays do not have alignment requirements.      */
/*                                                                          */
/*  NOTES                                                                   */
/*      The values of the two left-most and two right-most pixels on        */
/*       each line of the output are not well-defined.                      */
/*                                                                          */
/*  COMPATIBILITY                                                           */
/*      This code is compatible with C64x+ processors only.                 */
/*                                                                          */
/*  SOURCE                                                                  */
/*      The algorithm is taken from sobel in IMGLIB                         */
/* ======================================================================== */

#pragma CODE_SECTION(IMG_sobel_5x5_16s,   ".text:optimized");


void IMG_sobel_5x5_16s
(
    const short *restrict  in,   /* Input image data   */
          short *restrict out,   /* Output image data  */
          short          cols,   /* image columns      */
          short          rows    /* Image rows         */
)
{    

    int             i,             end;
    int             sum_h0,        sum_v0,      sum_h1;
    int             sum_v1,        o1,          o2;
    
    unsigned int    mask_01,       mask_34,     maskv3_10;
    unsigned int    maskv3_43;   
   

    long long       dp1,          dp2;
    long long       dpk1,         dpk2,         dpk3;
          
    long long       dt1,          dt2,          dt6;
    long long       dt3,          dt4,          dt5; 

    long long       row1_3210,    row2_3210,    row3_3210;
    long long       row4_3210,    row5_3210;
    
    unsigned int    row1_54,      row2_54,      row3_54;
    unsigned int    row4_54,      row5_54;

    /* -------------------------------------------------------------------- */
    /*  Uninitialise pointers to point to staring of each of the 5 rows     */
    /* -------------------------------------------------------------------- */
    const short *restrict in0;
    const short *restrict in1;
    const short *restrict in2;
    const short *restrict in3;
    const short *restrict in4;
     

    _nassert(cols                   > 5);  /* cols must be greater than 5 */
    _nassert(cols              % 2 == 0);  /* cols must be multiple of 2  */
    _nassert(rows                  >= 5);  /* rows must be atleast 5      */
    _nassert(cols * (rows - 4) - 4 >= 2);  /* atleast 2 output values     */
    
    
    
    
    /* -------------------------------------------------------------------- */
    /*  initialise pointers to point to staring of each of the 5 rows       */
    /* -------------------------------------------------------------------- */
    in0 = in;
    in1 = in0 + cols;
    in2 = in1 + cols;
    in3 = in2 + cols;
    in4 = in3 + cols;

    /* -------------------------------------------------------------------- */ 
    /*  Initialise mask values. Higher 16-bits have 2 packed vertical mask  */
    /*  elements(8-bit each) lower 16-bits have 2 horizontal mask elements  */
    /*  of the first column. Second row elements are at the LSB.mask_01 and */
    /*  mask_34 correspond to first 2 and last 2 elements of first column   */
    /*  repectively  of both the masks.                                     */ 
    /* -------------------------------------------------------------------- */
           
    mask_01   = 0x0104fffe ;    /* 1   4   -1   -2 */
    mask_34   = 0x04010201 ;    /* 4   1    2    1 */
    
    /* -------------------------------------------------------------------- */
    /*     Initialise mask variables for 3rd row of the vertical mask       */
    /* -------------------------------------------------------------------- */

    maskv3_10 = 0x000c0006 ;    /*    12         6 */
    maskv3_43 = 0xfffafff4 ;    /*    -6       -12 */ 
    

    /* -------------------------------------------------------------------- */
    /*                 Calculate the number of iterations                   */
    /* -------------------------------------------------------------------- */
    
    end       = (cols*(rows-4) - 4);
    
    
    for (i = 0; i < end; i += 2)
    {

       /*------------------------------------------------------------------ */
       /* load pixel values for row 1 and 2. calculate partial sum of       */
       /*  products for row 1 and 2.                                        */
       /*------------------------------------------------------------------ */
       row1_3210 = _mem8_const(in0    );
       row1_54   = _mem4_const(in0 + 4);

       row2_3210 = _mem8_const(in1    );
       row2_54   = _mem4_const(in1 + 4);

       /* ----------------------------------------------------------------- */
       /*   Pack the 1st and the 2nd row pixels of the 1st column  in an    */
       /*   integer using dpack2.                                           */
       /* ----------------------------------------------------------------- */
       dpk1      =  _dpack2(_loll(row1_3210) , _loll(row2_3210));
       dpk2      =  _dpack2(_hill(row1_3210) , _hill(row2_3210));
       dpk3      =  _dpack2(    row1_54    ,     row2_54   );

       /* ----------------------------------------------------------------- */
       /* multiply 1st and 2nd elements of 1st column of the 2 masks with   */
       /* 1st and 2nd row pixels.                                           */
       /* ----------------------------------------------------------------- */  
       dt1      =  _ddotp4( _loll(dpk1) , mask_01 );
       dt2      =  _ddotp4( _hill(dpk1) , mask_01 );
       dt3      =  _ddotp4( _loll(dpk2) , mask_01 );
       dt4      =  _ddotp4( _hill(dpk2) , mask_01 );
       dt5      =  _ddotp4( _loll(dpk3) , mask_01 );
       dt6      =  _ddotp4( _hill(dpk3) , mask_01 );

       
       /* ----------------------------------------------------------------- */
       /* Calculate partial sum for horizontal and vertical components h0,  */
       /*  h1 and v0, v1 respectively using first 2 rows of input image.    */
       /* ----------------------------------------------------------------- */

       sum_h0   =  _loll(dt1)       + (_loll(dt2) << 2) + (_loll(dt3) << 2) + 
                   (_loll(dt3) << 1) + (_loll(dt4) << 2) + _loll(dt5);

       sum_h1   =  _loll(dt2)       + (_loll(dt3) << 2) + (_loll(dt4) << 2) + 
                   (_loll(dt4) << 1) + (_loll(dt5) << 2) + _loll(dt6);

       sum_v0   =  _hill(dt1)       + (_hill(dt2) << 1) - (_hill(dt4) << 1) -
                   _hill(dt5); 

       sum_v1   =  _hill(dt2)      + (_hill(dt3) << 1) - (_hill(dt5) << 1) -
                   _hill(dt6); 

       in0      =  in0 + 2;
       in1      =  in1 + 2;

       /* ----------------------------------------------------------------- */
       /* Repeat the same process for the 4th and 5th rows                  */
       /* ----------------------------------------------------------------- */

       row4_3210 = _mem8_const(in3    );
       row4_54   = _mem4_const(in3 + 4);

       row5_3210 = _mem8_const(in4    );
       row5_54   = _mem4_const(in4 + 4);

       dpk1      = _dpack2(_loll(row4_3210) , _loll(row5_3210));
       dpk2      = _dpack2(_hill(row4_3210) , _hill(row5_3210));
       dpk3      = _dpack2(    row4_54    ,     row5_54   );

        
       dt1       = _ddotp4( _loll(dpk1) , mask_34 );
       dt2       = _ddotp4( _hill(dpk1) , mask_34 );
       dt3       = _ddotp4( _loll(dpk2) , mask_34 );
       dt4       = _ddotp4( _hill(dpk2) , mask_34 );
       dt5       = _ddotp4( _loll(dpk3) , mask_34 );
       dt6       = _ddotp4( _hill(dpk3) , mask_34 );


       
       sum_h0   +=  _loll(dt1)      + (_loll(dt2) << 2) + (_loll(dt3) << 2) + 
                   (_loll(dt3) << 1) + (_loll(dt4) << 2) + _loll(dt5);

       sum_h1   +=  _loll(dt2)      + (_loll(dt3) << 2) + (_loll(dt4) << 2) + 
                   (_loll(dt4) << 1) + (_loll(dt5) << 2) + _loll(dt6);
  
       sum_v0   +=  _hill(dt1)      + (_hill(dt2) << 1) - (_hill(dt4) << 1) -
                    _hill(dt5); 

       sum_v1   +=  _hill(dt2)      + (_hill(dt3) << 1) - (_hill(dt5) << 1) -
                    _hill(dt6); 

       in3       =  in3 + 2;
       in4       =  in4 + 2;
    
      
       /* ----------------------------------------------------------------- */
       /* load pixel values for row 3 and calculate partial sum of products */
       /* ----------------------------------------------------------------- */
    
       row3_3210 = _mem8_const(in2);
       row3_54   = _mem4_const(in2+4);
       
       dp1       = _ddotpl2(row3_3210 , maskv3_10);
       dp2       = _ddotph2(_itoll(row3_54,_hill(row3_3210)),maskv3_43);
       
       sum_v0    = sum_v0 + _loll(dp1) + _loll(dp2) ;
       sum_v1    = sum_v1 + _hill(dp1) + _hill(dp2) ;
       
       in2       = in2 + 2;

     

        
       /* ------------------------------------------------------------------ */
       /* claculate the output as the sum of absolute values of vertical and */
       /* horizontal components                                              */
       /* ------------------------------------------------------------------ */
     
       o1 = _abs(sum_h0) + _abs(sum_v0);
       o2 = _abs(sum_h1) + _abs(sum_v1);

       /* ----------------------------------------------------------------- */
       /* saturate and pack the result to store in the output array         */
       /* ----------------------------------------------------------------- */

       _mem4((void *)&(out[i+2])) = _spack2(o2,o1);   
   
    }                   
}

/* ======================================================================== */
/*  End of file:  IMG_sobel_5x5_16s.c                                       */
/* ======================================================================== */

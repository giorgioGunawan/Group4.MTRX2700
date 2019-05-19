# Group4.MTRX2700
G4-2700 Major Project

// To reverse strings
void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 

// Convert integer to string
int intToStr (int num, char str[], int d){
    
    // Set index
    int i = 0;
    
    // Take the mod and output to string (inverted)
    while(num != 0){
        str[i] = num%10 + '0';
    }
    
    while (i < d) 
        str[i++] = '0'; 
    
    // Reverse the string to obtain wanted char
    reverse(str,i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string. 
void floatToStr(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 

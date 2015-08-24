using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class degree : MonoBehaviour {
	SerialPort sp = new SerialPort("COM6", 9600);  //set Serial port
	int[] val = new int[5] ;
  int[] angle = new int[2] ;
  
  // Use this for initialization
	void Start() {
		sp.Open();  //Serial port open
		sp.ReadTimeout = 1;  //set Serial timeout
	}
  
	// Update is called once per frame
	void Update() {
		if (sp.IsOpen) {
			try {
        sp.Write("s") ;  //send start data
        val[0] = sp.ReadByte() ;  //read a byte
        if (val[0] == 0xff) {  //check start byte
          for (int i = 1; i < 5; i++) {
            val[i] = sp.ReadByte() ;
          }
          
          angle[0] = val[1] * (val[2] - 2) ;  //calculate value
          angle[1] = val[3] * (val[4] - 2) ;
          
          transform.rotation = Quaternion.Euler(angle[1], 0f, angle[0]) ;  //rotate cube
        }
			} 
      catch (System.Exception) {}
		}
	}
}
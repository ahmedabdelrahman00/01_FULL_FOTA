// For Firebase JS SDK v7.20.0 and later, measurementId is optional
// For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
  apiKey: "AIzaSyADCXJAtMSX0Fi8jtgz90yF-DepnijSogE",
  authDomain: "full-fota.firebaseapp.com",
  databaseURL: "https://full-fota-default-rtdb.firebaseio.com",
  projectId: "full-fota",
  storageBucket: "full-fota.appspot.com",
  messagingSenderId: "442262673816",
  appId: "1:442262673816:web:83fd86baf7d2998f4473ed",
  measurementId: "G-3BB8NP6WHM"
};
  
// Initialize Firebase
firebase.initializeApp(firebaseConfig);
const storage = firebase.storage();
const database = firebase.database();

var updatingformDB = firebase.database().ref("updating-form");

document.getElementById('updating-form').addEventListener("uppdate" , uploadFile );
function uploadFile() {
    console.log('Uploading file...');
  
    const fileInput = document.getElementById('file-input');
    const file = fileInput.files[0];
  
    if (file) {
      const storageRef = storage.ref();
      const fileRef = storageRef.child(file.name);
  
      fileRef.put(file).then(() => {
        console.log('File uploaded successfully!');
   //----------------------enable alert-----------------------
        }).catch((error) => {
         console.error('Error uploading file:', error);
        });
    } else {
         console.error('No file selected.');
        }
 }


  
 function uploudData(){
  var VERSIONvalue = document.getElementById("version").value;
   console.log("version : " , VERSIONvalue , ".  ");
   saveData(VERSIONvalue);
   alert('Updated Successfully');
}

const saveData = (VERSIONvalue) => {
  // Parse VERSIONvalue to integer
  const versionInt = parseInt(VERSIONvalue);
  
  // Check if versionInt is a valid integer
  if (!isNaN(versionInt)) {
    updatingformDB.set({
      Version: versionInt
    });
  } else {
    console.error('Invalid version number:', VERSIONvalue);
  }
}

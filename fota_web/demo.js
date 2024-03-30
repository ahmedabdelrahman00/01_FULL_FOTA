// For Firebase JS SDK v7.20.0 and later, measurementId is optional
// For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
  apiKey: "AIzaSyDdXhbBrDCZwZ08M9oOJKfy_7_05-s9TbY",
  authDomain: "update-f19b4.firebaseapp.com",
  databaseURL: "https://update-f19b4-default-rtdb.firebaseio.com",
  projectId: "update-f19b4",
  storageBucket: "update-f19b4.appspot.com",
  messagingSenderId: "1040245004423",
  appId: "1:1040245004423:web:1b4d05f8332d82eedf0022",
  measurementId: "G-ZC084MB0V9"
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
   var ECUvalue = document.getElementById("ECU").value;
   var VERSIONvalue = document.getElementById("version").value;
    console.log("Node : " , ECUvalue , ", Version : " , VERSIONvalue , ".  ");
    saveData(VERSIONvalue , ECUvalue);
    alert('Updated Successfully');
}

const saveData = (VERSIONvalue , ECUvalue) => {
  if(ECUvalue=="ECU 1"){
    updatingformDB.child('Node_1').set({
   Version : VERSIONvalue,
  });
  }else if(ECUvalue=="ECU 2"){
  updatingformDB.child('Node_2').set({
  Version : VERSIONvalue,
  });}
  else if(ECUvalue=="ECU 3"){
  updatingformDB.child('Node_3').set({
    Version : VERSIONvalue,
   });}

}


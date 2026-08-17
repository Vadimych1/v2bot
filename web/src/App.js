import './App.css';
import { Button, DictDisplay } from './widgets/Interactive';

function App() {
  return (
    <div className="App">
      <Button>My own button</Button>

      <DictDisplay hint='test values'>{{ "a": "b", "c": "d" }}</DictDisplay>
    </div>
  );
}

export default App;
